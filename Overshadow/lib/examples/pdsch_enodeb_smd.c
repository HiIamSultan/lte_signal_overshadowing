#include <uhd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/select.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <srsran/phy/common/phy_common.h>
#include <srsran/phy/phch/pdsch_cfg.h>
#include <getopt.h>
#include "srsran/srsran.h"

#define UE_CRNTI 0x1234
#define M_CRNTI 0xFFFD

#ifndef DISABLE_RF
#include "srsran/phy/rf/rf.h"
#include "srsran/phy/rf/rf_utils.h"
#include "srsran/phy/common/phy_common.h"
#include "../src/phy/rf/rf_uhd_imp.h"

srsran_rf_t rf;
#else
#warning Compiling pdsch_ue with no RF support
#endif

char *output_file_name = NULL;
float rf_amp = 0.8, rf_gain = 15.0, rf_freq = 2400000000;
char *rf_args = "";
srsran_ue_sync_t ue_sync;
srsran_ue_mib_t ue_mib;
srsran_pbch_t pbch;
srsran_ofdm_t ifft[SRSRAN_MAX_PORTS];
cf_t *sf_buffer[SRSRAN_MAX_PORTS] = {NULL}, *sf_buffer_o[SRSRAN_MAX_PORTS] = {NULL}, *output_buffer[SRSRAN_MAX_PORTS] = {NULL};
int sf_n_re, sf_n_samples;
srsran_timestamp_t last_stamp;
pthread_t tx_thread, rx_thread;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
bool go_exit = false;
bool cell_found = false;
bool mib_observed = false;

char* rf_dev = "";
int force_N_id_2 = -1;

bool use_standard_lte_rate = false;
cf_t* sf_symbols[SRSRAN_MAX_PORTS];

uint32_t sfn;
uint32_t first_sfn;
int rx_ret = -1;
uint32_t sf_idx;

srsran_cell_t cell = {
    .nof_prb = 25,
    .nof_ports = 1,
    .id = 0,
    .cp = SRSRAN_CP_NORM,
    .phich_length = SRSRAN_PHICH_NORM,
    .phich_resources = SRSRAN_PHICH_R_1,
    .frame_type = SRSRAN_FDD,
};

cell_search_cfg_t cell_detect_config = {
    .max_frames_pbch      = SRSRAN_DEFAULT_MAX_FRAMES_PBCH,
    .max_frames_pss       = SRSRAN_DEFAULT_MAX_FRAMES_PSS,
    .nof_valid_pss_frames = SRSRAN_DEFAULT_NOF_VALID_PSS_FRAMES,
    .init_agc             = 0,
    .force_tdd            = false
};

void usage(char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\t-a RF args [Default %s]\n", rf_args);
    printf("\t-g RF TX gain [Default %.2f dB]\n", rf_gain);
    printf("\t-f RF TX frequency [Default %.1f MHz]\n", rf_freq / 1000000);
    printf("\t-l RF amplitude [Default %.2f]\n", rf_amp);
    printf("\t-c Cell ID [Default %d]\n", cell.id);
    printf("\t-p Number of PRBs [Default %d]\n", cell.nof_prb);
    printf("\t-P Number of PRBs [Default %d]\n", cell.nof_ports);
    printf("\t-h Show this help message\n");
}

void parse_args(int argc, char **argv) {
  int opt;
  while ((opt = getopt(argc, argv, "a:g:f:l:c:p:P:h")) != -1) {
    switch (opt) {
      case 'a':
        if (optarg) {
          rf_args = optarg;
        } else {
          fprintf(stderr, "Option -a requires an argument.\n");
          usage(argv[0]);
          exit(-1);
        }
        break;
      case 'g':
        if (optarg) {
          rf_gain = strtof(optarg, NULL);
        } else {
          fprintf(stderr, "Option -g requires an argument.\n");
          usage(argv[0]);
          exit(-1);
        }
        break;
      case 'f':
        if (optarg) {
          rf_freq = strtof(optarg, NULL);
        } else {
          fprintf(stderr, "Option -f requires an argument.\n");
          usage(argv[0]);
          exit(-1);
        }
        break;
      case 'l':
        if (optarg) {
          rf_amp = strtof(optarg, NULL);
        } else {
          fprintf(stderr, "Option -l requires an argument.\n");
          usage(argv[0]);
          exit(-1);
        }
        break;
      case 'c':
        if (optarg) {
          cell.id = (uint32_t)strtol(optarg, NULL, 10);
        } else {
          fprintf(stderr, "Option -c requires an argument.\n");
          usage(argv[0]);
          exit(-1);
        }
        break;
      case 'p':
        if (optarg) {
          cell.nof_prb = (uint32_t)strtol(optarg, NULL, 10);
        } else {
          fprintf(stderr, "Option -p requires an argument.\n");
          usage(argv[0]);
          exit(-1);
        }
        break;
      case 'P':
        if (optarg) {
          cell.nof_ports = (uint32_t)strtol(optarg, NULL, 10);
        } else {
          fprintf(stderr, "Option -P requires an argument.\n");
          usage(argv[0]);
          exit(-1);
        }
        break;
      case 'h':
      default:
        usage(argv[0]);
        exit(-1);
    }
  }
}

void sig_int_handler(int signo) {
    printf("SIGINT received. Exiting...\n");
    if (signo == SIGINT) {
        go_exit = true;
        pthread_cond_signal(&cond);
    }
}

void initialize() {
    // Initialize UE synchronization and MIB decoder
    printf("Initializing UE sync and MIB decoder\n");

    // Allocate memory for subframe buffers
    sf_n_re = 2 * SRSRAN_CP_NORM_NSYMB * cell.nof_prb * SRSRAN_NRE;
    sf_n_samples = 2 * SRSRAN_SLOT_LEN(srsran_symbol_sz(cell.nof_prb));

    for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
        sf_buffer_o[i] = srsran_vec_cf_malloc(sizeof(cf_t) * sf_n_re);
        if (!sf_buffer_o[i]) {
            perror("malloc");
            exit(-1);
        }
        output_buffer[i] = srsran_vec_cf_malloc(sizeof(cf_t) * sf_n_samples);
        if (!output_buffer[i]) {
            perror("malloc");
            exit(-1);
        }
    }

    for (int i = 0; i < cell.nof_ports; i++) {
        if (srsran_ofdm_tx_init(&ifft[i], SRSRAN_CP_NORM, sf_buffer[i], output_buffer[i], cell.nof_prb)) {
            fprintf(stderr, "Error initializing OFDM TX\n");
            exit(-1);
        }
        srsran_ofdm_set_normalize(&ifft[i], true);
    }
    
    if (srsran_pbch_init(&pbch)) {
        fprintf(stderr, "Error initializing PBCH\n");
        exit(-1);
    }
    if (srsran_pbch_set_cell(&pbch, cell)) {
        fprintf(stderr, "Error setting PBCH cell\n");
        exit(-1);
    }
}

void cleanup() {
    srsran_pbch_free(&pbch);
    for (int i = 0; i < cell.nof_ports; i++) {
      srsran_ofdm_tx_free(&ifft[i]);
    }
    for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
        if (sf_buffer_o[i]) {
            free(sf_buffer_o[i]);
        }
        if (output_buffer[i]) {
            free(output_buffer[i]);
        }
    }
}

int srsran_rf_recv_wrapper(void* h, cf_t* data_[SRSRAN_MAX_PORTS], uint32_t nsamples, srsran_timestamp_t *t) {
    DEBUG(" ----  Receive %d samples  ----", nsamples);
    void* ptr[SRSRAN_MAX_PORTS];
    for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
        ptr[i] = data_[i];
    }
    return srsran_rf_recv_with_time_multi(h, ptr, nsamples, true, &t->full_secs, &t->frac_secs);
}

static SRSRAN_AGC_CALLBACK(srsran_rf_set_rx_gain_th_wrapper_)
{
  srsran_rf_set_rx_gain_th((srsran_rf_t*)h, gain_db);
}

void *rx_thread_func() {
    printf("[Rx] Receiver Started\n");
    unsigned long mask = 8; // processor 4   // 1 2 4 8 (1,2,3,4)
    if (pthread_setaffinity_np(pthread_self(), sizeof(mask), (cpu_set_t *)&mask) < 0) {
      perror("pthread_setaffinity_np");
    }

    int ret;
    float search_cell_cfo = 0;
    int sfn_offset;

  //#ifndef DISABLE_RF
    printf("Opening RF device with %d RX antennas...\n", cell.nof_ports);
    if (srsran_rf_open_devname(&rf, rf_dev, rf_args, cell.nof_ports)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    /* Set receiver gain */
    printf("Starting AGC thread...\n");
    if (srsran_rf_start_gain_thread(&rf, false)) {
      ERROR("Error opening rf");
      exit(-1);
    }
    srsran_rf_set_rx_gain(&rf, srsran_rf_get_rx_gain(&rf));
    cell_detect_config.init_agc = srsran_rf_get_rx_gain(&rf);

    /* set receiver frequency */
    printf("Tunning receiver to %.3f MHz\n", rf_freq / 1000000);
    srsran_rf_set_rx_freq(&rf, cell.nof_ports, rf_freq);

    uint32_t ntrial = 0;
    do {
      ret = rf_search_and_decode_mib(
          &rf, cell.nof_ports, &cell_detect_config, force_N_id_2, &cell, &search_cell_cfo);
      if (ret < 0) {
        ERROR("Error searching for cell");
        exit(-1);
      } else if (ret == 0 && !go_exit) {
        printf("Cell not found after %d trials. Trying again (Press Ctrl+C to exit)\n", ntrial++);
      }
    } while (ret == 0 && !go_exit);

    /* set sampling frequency */
    int srate = srsran_sampling_freq_hz(cell.nof_prb);
    if (srate != -1) {
      printf("Setting sampling rate %.2f MHz\n", (float)srate / 1000000);
      float srate_rf = srsran_rf_set_rx_srate(&rf, (double)srate);
      if (srate_rf != srate) {
        ERROR("Could not set sampling rate");
        exit(-1);
      }
    } else {
      ERROR("Invalid number of PRB %d", cell.nof_prb);
      exit(-1);
    }

    INFO("Stopping RF and flushing buffer...\r");
  //#endif

  //#ifndef DISABLE_RF
    int decimate = 0;
    if (srsran_ue_sync_init_multi_decim(&ue_sync,
                                        cell.nof_prb,
                                        cell.id == 1000,
                                        srsran_rf_recv_wrapper,
                                        cell.nof_ports,
                                        (void*)&rf,
                                        decimate)) {
        ERROR("Error initiating ue_sync");
        exit(-1);
    }
    if (srsran_ue_sync_set_cell(&ue_sync, cell)) {
      ERROR("Error initiating ue_sync");
      exit(-1);
    }
  //#endif
    uint32_t max_num_samples = 3 * SRSRAN_SF_LEN_PRB(cell.nof_prb); /// Length in complex samples
    for (int i = 0; i < cell.nof_ports; i++) {
      sf_buffer[i] = srsran_vec_cf_malloc(max_num_samples);
    }
    srsran_ue_mib_t ue_mib;
    if (srsran_ue_mib_init(&ue_mib, sf_buffer[0], cell.nof_prb)) {
      ERROR("Error initaiting UE MIB decoder");
      exit(-1);
    }
    if (srsran_ue_mib_set_cell(&ue_mib, cell)) {
      ERROR("Error initaiting UE MIB decoder");
      exit(-1);
    }

  //#ifndef DISABLE_RF
    srsran_rf_start_rx_stream(&rf, false);
  //#endif

  //#ifndef DISABLE_RF
    srsran_rf_info_t* rf_info = srsran_rf_get_info(&rf);
    srsran_ue_sync_start_agc(&ue_sync,
                             srsran_rf_set_rx_gain_th_wrapper_,
                             rf_info->min_rx_gain,
                             rf_info->max_rx_gain,
                             cell_detect_config.init_agc);
  //#endif

    ue_sync.cfo_correct_enable_track = true;

    srsran_pbch_decode_reset(&ue_mib.pbch);

    INFO("\nEntering main loop...");

    // Variables for measurements
    float sinr[SRSRAN_MAX_LAYERS][SRSRAN_MAX_CODEBOOKS] = {};

    for (int i = 0; i < SRSRAN_MAX_LAYERS; i++) {
      srsran_vec_f_zero(sinr[i], SRSRAN_MAX_CODEBOOKS);
    }

    /* Main loop */
    while (!go_exit) {

      pthread_mutex_lock(&mutex);
      cf_t* buffers[SRSRAN_MAX_CHANNELS] = {};

      for (int p = 0; p < SRSRAN_MAX_PORTS; p++) {
          buffers[p] = sf_buffer[p];
      }
      ret = srsran_ue_sync_zerocopy(&ue_sync, buffers, max_num_samples);

      if (ret != 1) {
        rx_ret = -1;
        sfn = -100;
        fprintf(stderr,"zerocopy_multi failed\n");
        pthread_mutex_unlock(&mutex);
      }

      if (ret == 1) {
        rx_ret = 0;
        cell_found = true;
        pthread_cond_signal(&cond); // Signal to the tx_thread_func that cell_found is updated
        srsran_ue_sync_get_last_timestamp(&ue_sync, &last_stamp);
        sf_idx = srsran_ue_sync_get_sfidx(&ue_sync);
        if (srsran_ue_sync_get_sfidx(&ue_sync) == 0 && !mib_observed) {
          uint8_t bch_payload[SRSRAN_BCH_PAYLOAD_LEN];
          int n = srsran_ue_mib_decode(&ue_mib, bch_payload, NULL, &sfn_offset);
          if (n < 0) {
            printf("Error decoding UE MIB\n");
            exit(-1);
          } else if (n == 0) {
            sfn = -100;
            printf("MIB DECODING FAILED\n");
          } else if (n == SRSRAN_UE_MIB_FOUND)  {
            srsran_pbch_mib_unpack(bch_payload, &cell, &sfn);
            srsran_cell_fprint(stdout, &cell, sfn);
            printf("Received MIB: SFN=%u, Cell ID=%d, PRBs=%d, Ports=%d\n", sfn, cell.id, cell.nof_prb, cell.nof_ports);
            sfn = (sfn + sfn_offset) % 1024;
            first_sfn = sfn;
            mib_observed = true;
          }
        }
        pthread_mutex_unlock(&mutex);
      }
    }

    for (int i = 0; i < cell.nof_ports; i++) {
      if (sf_buffer[i]) {
        free(sf_buffer[i]);
      }
    }

    srsran_ue_mib_free(&ue_mib);
    return NULL;
}

void *tx_thread_func() {
    printf("[Tx] Transmitter Started\n");
    unsigned long mask = 8; // processor 4   // 1 2 4 8 (1,2,3,4)
    if (pthread_setaffinity_np(pthread_self(), sizeof(mask), (cpu_set_t *)&mask) < 0) {
      perror("pthread_setaffinity_np");
    }

    while (true) {
      pthread_mutex_lock(&mutex);
      while (!cell_found && !go_exit) {
        pthread_cond_wait(&cond, &mutex); // Wait for the signal from rx_thread_func
      }
      pthread_mutex_unlock(&mutex);

      if (go_exit) break;

      printf("[Tx] Cell Found\n");
      srsran_timestamp_t future_time, cur_time;
      bool start_of_burst = true;
      bool first = true;
      float time_offset = 0.01 - 0.0001;

      uint32_t cur_sf_idx;
      uint32_t mib_sfn;
      uint32_t cur_sfn;
      uint32_t target_sfn = -1;
      int cur_rx_ret;
      int samp_rate;

      float estimated_cfo, estimated_sfo;

      cf_t pss_signal[SRSRAN_PSS_LEN];
      float sss_signal0[SRSRAN_SSS_LEN], sss_signal5[SRSRAN_SSS_LEN];
      uint8_t bch_payload[SRSRAN_BCH_PAYLOAD_LEN];
      
      srsran_use_standard_symbol_size(use_standard_lte_rate);

      sf_n_re = SRSRAN_SF_LEN_RE(cell.nof_prb, cell.cp);
      sf_n_samples = 2 * SRSRAN_SLOT_LEN(srsran_symbol_sz(cell.nof_prb));

      initialize();
      printf("[Tx] Initialized\n");

      srsran_pss_generate(pss_signal, cell.id % 3);
      srsran_sss_generate(sss_signal0, sss_signal5, cell.id);

      for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
        sf_symbols[i] = sf_buffer[i % cell.nof_ports];
      }

    //#ifndef DISABLE_RF
      int srate = srsran_sampling_freq_hz(cell.nof_prb);
      if (srate != -1) {
        printf("Setting sampling rate %.2f MHz\n", (float)srate / 1000000);
        float srate_rf = srsran_rf_set_tx_srate(&rf, (double)srate);
        if (srate_rf != srate) {
          ERROR("Could not set sampling rate");
          exit(-1);
        }
      } else {
        ERROR("Invalid number of PRB %d", cell.nof_prb);
        exit(-1);
      }
      srsran_rf_set_tx_gain(&rf, rf_gain);
      printf("Set TX gain: %.1f dB\n", srsran_rf_get_tx_gain(&rf));
      printf("Set TX freq: %.2f MHz\n", srsran_rf_set_tx_freq(&rf, cell.nof_ports, rf_freq) / 1000000);
    //#endif

      while (!go_exit) {
        pthread_mutex_lock(&mutex);

        cur_sf_idx = sf_idx;
        mib_sfn = first_sfn - sfn + 1024;
        cur_sfn = sfn;
        cur_rx_ret = rx_ret;
        memcpy(&cur_time, &last_stamp, sizeof(srsran_timestamp_t));
        pthread_mutex_unlock(&mutex);

        // Debug prints
        //printf("cur_rx_ret: %d, mib_sfn: %u, cur_sf_idx: %u, mib_observed: %d\n", cur_rx_ret, mib_sfn, cur_sf_idx, mib_observed);

        if (cur_rx_ret != 0) {
          printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![Tx] rx_ret: %d\n",cur_rx_ret);
          fprintf(stderr,"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![Tx] rx_ret: %d\n",cur_rx_ret);
        }
        if (cur_rx_ret == 0 && first == true) {
          samp_rate = srsran_sampling_freq_hz(cell.nof_prb);
          estimated_cfo = srsran_ue_sync_get_cfo(&ue_sync);
          estimated_sfo = srsran_ue_sync_get_sfo(&ue_sync);
          first = false;
          printf("Frequency offset estimated..........CFO: %f SFO: %f\n",estimated_cfo,estimated_sfo);
          continue;
        }

        if (cur_rx_ret == 0 && (mib_sfn % 4) == 3 && cur_sf_idx == 0 && mib_observed) {
          memcpy(&future_time, &cur_time, sizeof(srsran_timestamp_t));
          future_time.frac_secs += time_offset;
          future_time.frac_secs -= (60.0/15360000.0); // 15.36 commercial eNodeB: KT 379 954.3e6

          if (future_time.frac_secs >= 1.0) {
            future_time.full_secs += (int) future_time.frac_secs;
            future_time.frac_secs -= (int) future_time.frac_secs;
          }
          target_sfn = cur_sfn + 4;
          target_sfn = target_sfn % 1024;

          int ret = -1;

          for (int idx = 0; idx < 10; idx++) {
            srsran_vec_cf_zero(sf_symbols[0], sf_n_re);

            if (idx == 0 || idx == 5) {
              srsran_pss_put_slot(pss_signal, sf_symbols[0], cell.nof_prb, cell.cp);
              srsran_sss_put_slot(sf_idx ? sss_signal5 : sss_signal0, sf_symbols[0], cell.nof_prb, cell.cp);
            }

            for (int i = 1; i < cell.nof_ports; i++) {
              memcpy(sf_symbols[i], sf_symbols[0], sizeof(cf_t) * sf_n_re);
            }

            srsran_cell_t cell_o;
            cell_o = cell;
            cell_o.nof_ports = 3;
            cell_o.phich_length = SRSRAN_PHICH_R_2;

            srsran_pbch_mib_pack(&cell_o, target_sfn, bch_payload);
            if (sf_idx == 0) {
              srsran_pbch_encode(&pbch, bch_payload, sf_buffer, target_sfn % 4);
            }

            for (int i = 0; i < cell.nof_ports; i++) {
              srsran_ofdm_tx_sf(&ifft[i]);
            }

            float norm_factor = (float)cell.nof_prb / 15 / sqrtf(1);
            for (int i = 0; i < cell.nof_ports; i++) {
              srsran_vec_sc_prod_cfc(output_buffer[i], rf_amp * norm_factor, output_buffer[i], SRSRAN_SF_LEN_PRB(cell.nof_prb));
            }

          }
          if (cur_sf_idx == 0 && (mib_sfn % 4) == 0 && mib_observed) {
            printf("[Subframe %d] [future_time] next_sfn: %d %.f: %f s\n", sf_idx, target_sfn, difftime(future_time.full_secs, (time_t) 0),future_time.frac_secs);
            
            srsran_rf_send_timed_multi(&rf, (void**)output_buffer, sf_n_samples, future_time.full_secs, future_time.frac_secs, true, start_of_burst, false);
            start_of_burst=false;
            printf("Overshadowing: SFN=%u, Subframe=%d\n", target_sfn, sf_idx);

            if (ret != sf_n_samples*1.2) {
              printf("[!] Warning!!!!!!!!!: txd sample is not sf_n_samples*1.2!!!!!\n");
              exit(-1);
            }
          }
          first = false;
        }

        /*if (mib_sfn % 16 == 0) {
          mib_observed = false;
        }*/
      }
    }
    cleanup();
    return NULL;
}

int main(int argc, char **argv) {
    // Parse arguments
    parse_args(argc, argv);

    // Initialize signal handlers
    signal(SIGINT, sig_int_handler);

    // Configure RF parameters
    if (srsran_rf_open_multi(&rf, rf_args, cell.nof_ports)) {
        fprintf(stderr, "Error opening RF\n");
        exit(-1);
    }

    // Start RX and TX threads
    if (pthread_create(&rx_thread, NULL, rx_thread_func, NULL)) {
        perror("pthread_create");
        exit(-1);
    }
    if (pthread_create(&tx_thread, NULL, tx_thread_func, NULL)) {
        perror("pthread_create");
        exit(-1);
    }

    // Wait for threads to complete
    int status;
    printf("before\n");
    pthread_join(rx_thread, NULL);
    pthread_join(tx_thread, NULL);

    status = pthread_mutex_destroy(&mutex);
    srsran_ue_sync_free(&ue_sync);
    srsran_rf_close(&rf);
    printf("code  =  %d\n", status);
    printf("PROGRAM END\n");
    return 0;
}