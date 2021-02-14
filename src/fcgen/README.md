This tool uses Python cogapp to auto generate class definitions and boilerplate code

You absolutely do not need to use this to generate new class definitions, but given the scope of 
fastcat covers ~20 devices and 3 implementations, this tool could be a big time saver

To install cogapp:

```bash
  sudo pip install cogapp
```

To use this tool, follow these steps

1) define the input file, use the el2124 example file
1) invoke the cog program with the following command, changing the path to your YAML file accordingly

```bash
cog -d -D yaml_file=/home/dev/src/fastcat/fastcat_device/tools/cog_workspace/input/el2124_slave.yaml @cog_in.txt
```
1) copy the output/ac_class.h/.cc files to the proper locations
1) copy-paste the snippet code to the proper locations
1) make sure it compiles
1) fill out the application-specific device Read() and Write() functions
