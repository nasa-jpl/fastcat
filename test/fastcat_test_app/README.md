# fastcat Test Application

Minimal test application to ensure the library presents to applications as expected.

Test it out with:
``` bash
mkdir build
cd build
cmake ..
make
```
Feel free to tailor the CMakeLists.txt to suit your testing needs. Leaving it pinned to master branch since that's the only branch that is guaranteed to always exist.

Here's an example output of expected output for an application that is trying to improperly use the private and protected methods or the interally build `fastcat-test` library


``` text
# Test that the protected and private overrides work properly

/home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/ftest_invalid_access.cc: In function ‘int main()’:
/home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/ftest_invalid_access.cc:9:15: error: ‘double fastcat::Actuator::CntsToEu(int32_t)’ is protected within this context
    9 |   act.CntsToEu(100); // protected
      |   ~~~~~~~~~~~~^~~~~
In file included from /home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/build/include/fastcat/manager.h:16,
                 from /home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/build/include/fastcat/fastcat.h:4,
                 from /home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/ftest_invalid_access.cc:2:
/home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/build/include/fastcat/jsd/actuator.h:102:11: note: declared protected here
  102 |   double  CntsToEu(int32_t cnts);
      |           ^~~~~~~~
/home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/ftest_invalid_access.cc:10:18: error: ‘bool fastcat::Actuator::prof_pos_hold_’ is private within this context
   10 |   bool val = act.prof_pos_hold_; // private
      |                  ^~~~~~~~~~~~~~
In file included from /home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/build/include/fastcat/manager.h:16,
                 from /home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/build/include/fastcat/fastcat.h:4,
                 from /home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/ftest_invalid_access.cc:2:
/home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/build/include/fastcat/jsd/actuator.h:183:8: note: declared private here
  183 |   bool prof_pos_hold_;
      |        ^~~~~~~~~~~~~~
make[2]: *** [CMakeFiles/ftest-invalid-access.dir/build.make:76: CMakeFiles/ftest-invalid-access.dir/ftest_invalid_access.cc.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:223: CMakeFiles/ftest-invalid-access.dir/all] Error 2
make: *** [Makefile:136: all] Error 2

```

And this is the output for the 2nd executable `ftest-invalid-access-lib` which aims to link against the library `fastcat-test` instead of the proper `fastcat` target library:
```
# Test that the fastcat-test lib is not available to applications by accident

[ 94%] Building CXX object CMakeFiles/ftest-invalid-access-lib.dir/ftest_invalid_access.cc.o
/home/abrinkma/src/fastcat_ws/fastcat-uts/test/fastcat_test_app/ftest_invalid_access.cc:2:10: fatal error: fastcat/fastcat.h: No such file or directory
    2 | #include "fastcat/fastcat.h"
      |          ^~~~~~~~~~~~~~~~~~~
compilation terminated.
make[2]: *** [CMakeFiles/ftest-invalid-access-lib.dir/build.make:76: CMakeFiles/ftest-invalid-access-lib.dir/ftest_invalid_access.cc.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:217: CMakeFiles/ftest-invalid-access-lib.dir/all] Error 2
make: *** [Makefile:136: all] Error 2
```
