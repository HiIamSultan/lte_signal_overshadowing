# Description

This repository (signal overshadowing attack) exploits the fundamental weakness of broadcast messages in LTE and modifies a transmitted signal over the air. By using this tool, you can inject a manipulated MIB signal into UEs without employing an FBS. And if you modify the code, you can also inject other broadcast messages such as SIBs and paging.

### Use responsively: Overshadowing a LTE signal on licensed frequencies might be illegal in your country. (Each country has unique regulations regarding the wireless transmission of signals and these regulations are drafted, implemented, and modified by each country's government, not by an international organization.) Also, it may affect other users not to receive legitimate signals.

## User Manual
This repository is implemented on the top of the srsRAN 4G. So it requires the same setup as when executing the srsRAN 4G.

### Prerequisite
1. USRP B210
2. OctoClock (+It’s better to have GPS) to get better time syncronization for all the USRPs.
3. Ubuntu PC (I used 22.04) that can execute the srsRAN 4G.
4. Configuration of target eNB. (e.g. by using pdsch_ue of srsRAN 4G)
5. Manipulated MIB message.

### Overall Procedure
I have conducted my experiments on testbed based LTE cellular network, particularly on srsRAN 4G eNB and UE. Hence, I have used three USRPs, one B210 for srsRAN 4G eNB (particularly I have run the pdsch_enodeb), one B210 for srsRAN 4G UE (particularly I have run the pdsch_ue), one B210 for the injector (pdsch_enodeb_o or pdsch_enodeb_smd or pdsch_enodeb_mqm). I have also collected the traces over the air and observed the spectrum using [inspectrum](https://github.com/miek/inspectrum).

Building

mkdir build
cd build
cmake ..
make

Executing

cd lib/examples
sudo ./pdsch_enodeb_o -f @1 -p @2 -g @3 -c @4
@1: TX frequency (target cell's DL frequency)
@2: number of prb
@3: TX gain
@4: modified cell id

## Over The Air Network Traces in Inspectrum
Without injected MIB message - 

With injected MIB message - 
![image](https://github.com/user-attachments/assets/30e9470f-28b7-492d-a805-313a642c16b5)

Credits
I sincerely appreciate the [SRS](https://srs.io/) team for making their great software available and SysSec-KAIST group for making their [sigover](https://github.com/SysSec-KAIST/sigover_injector?tab=readme-ov-file) tool public.

Please refer to the [Sigover](https://syssec.kaist.ac.kr/pub/2019/sec19-yang-hojoon.pdf) paper for more details.
@article{yaang:2019:sigover,
  author = {Yang, Hojoon and Bae, Sangwook and Son, Mincheol and Kim, Hongil and Kim, Song Min and Kim, Yongdae},
  title = {Hiding in plain signal: Physical signal overshadowing attack on {LTE}},
  booktitle = {28th USENIX Security Symposium (USENIX Security 19)},
  year = 2019
}
