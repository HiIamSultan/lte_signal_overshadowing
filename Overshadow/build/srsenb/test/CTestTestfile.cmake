# CMake generated Testfile for 
# Source directory: /home/cseuser/Desktop/Overshadow/srsenb/test
# Build directory: /home/cseuser/Desktop/Overshadow/build/srsenb/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(enb_metrics_test "enb_metrics_test" "-o" "/home/cseuser/Desktop/Overshadow/build/srsenb/test/enb_metrics.csv")
set_tests_properties(enb_metrics_test PROPERTIES  _BACKTRACE_TRIPLES "/home/cseuser/Desktop/Overshadow/srsenb/test/CMakeLists.txt;29;add_test;/home/cseuser/Desktop/Overshadow/srsenb/test/CMakeLists.txt;0;")
subdirs("mac")
subdirs("phy")
subdirs("upper")
subdirs("rrc")
subdirs("s1ap")
