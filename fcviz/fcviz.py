#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.8"
# dependencies = [
#   "graphviz",
#   "ipython==7.9",
#   "PyYAML",
# ]
# ///
# If you prefer not to use uv, install the dependencies listed above and run
# this file with python3 directly.

import argparse

def ParseArgs():
  parser = argparse.ArgumentParser(
      description='Generate a Graphviz diagram from a fastcat configuration YAML file.')
  parser.add_argument(
      'config_yaml',
      help='Path to the fastcat configuration YAML file.')
  return parser.parse_args()

def snake2camel(value):
  return ''.join(x.capitalize() or '_' for x in value.split('_'))

def GetCommandFields(type_data, input_command):
  field_list = []
  for command in type_data['commands']:
    print("checking command: %s against input: %s" % (command['name'] + '_cmd', input_command.lower()))
    if command['name'] + '_cmd' == input_command.lower():
      print("matched %s, now appending fields..." % command['name'] + '_cmd')
      fields = command['fields']
      for field in fields:
        field_list.append(field['name'])

  return field_list

def GetStateFields(type_data, device_class_name):
  states = type_data['states']
  field_list = []
  for state in states:
    camel = snake2camel(state['name'])
    print("checking state: %s against input: %s" % (camel, device_class_name))
    if camel == device_class_name:
      print("matched %s, now appending fields..." % state['name'])
      fields = state['fields']
      for field in fields:
        field_list.append(field['name'])

  return field_list

def CreateGraph(bus_data, type_data, Digraph):
  dot = Digraph(comment="fcviz")
  dot.graph_attr['nodesep'] = '1'
  dot.node_attr['shape'] = 'record'
  dot.node_attr['width'] = '0.1'
  dot.node_attr['height'] = '0.1'

  for bus in bus_data:
    devices = bus['devices']
    for dev in devices:
      ## Create a New node for the device
      if dev['device_class'] == "IGNORE":
        continue
      elif dev['device_class'] == "Commander":
        fields = GetCommandFields(type_data, dev['device_cmd_type'])
        node_label = dev['device_class'] + ' | <' + dev['name']  + '> ' + dev['name']
        node_label = node_label + ' | <enable> enable'
        node_label = node_label + ' | ' + dev['device_cmd_type'].upper()
        for field in fields:
          node_label = node_label + ' | <' + field + '> ' + field;
        print(node_label)
        node_label = '{' + node_label + '}'
        print(node_label)
        dot.node(dev['name'], label=node_label)
      else:
        fields = GetStateFields(type_data, dev['device_class'])
        node_label = dev['device_class'] + ' | <' + dev['name']  + '> ' + dev['name']
        for field in fields:
          node_label = node_label + ' | <' + field + '> ' + field;
        print(node_label)
        node_label = '{' + node_label + '}'
        print(node_label)
        dot.node(dev['name'], label=node_label)

      ## Create edges for signals

      if dev['device_class'] == "Commander":
        dot.edge(dev['name'], dev['device_cmd_name'], label=dev['device_cmd_type'])

      if('signals' in dev):
        n_sig = 0
        for sig in dev['signals']:
          n_sig = n_sig+1

          if sig['observed_device_name'] == 'FIXED_VALUE':
            cmder = dev['name']
            if 'cmd_field_name' in sig:
              field = sig['cmd_field_name']
            else:
              field = str(n_sig)
            val = str(sig['fixed_value'])
            from_node_str = cmder+'_'+field
            dot.node(from_node_str, label=val)
            #label_str = from_node_str + " = " + val
            label_str = ''
            to_node_str = dev['name'] + ':' + field
          elif dev['device_class'] == 'Commander':
            from_node_str = sig['observed_device_name'] + ':' + sig['request_signal_name']
            label_str = sig['request_signal_name']
            to_node_str = dev['name'] + ':' + sig['cmd_field_name']
          else:
            from_node_str = sig['observed_device_name'] + ':' + sig['request_signal_name']
            label_str = sig['request_signal_name']
            to_node_str = dev['name'] + ':' + dev['name']

          print('from: %s; to: %s' % (from_node_str, to_node_str))
          dot.edge(from_node_str, to_node_str, label=label_str)

  return dot

def main():
  args = ParseArgs()

  from graphviz import Digraph
  import yaml
  import IPython

  print("Attempting to open %s" % args.config_yaml)
  with open(args.config_yaml, 'r') as config_file:
    data = yaml.load(config_file, Loader=yaml.Loader)

  with open('src/fcgen/fastcat_types.yaml', 'r') as types_file:
    type_data = yaml.load(types_file, Loader=yaml.Loader)

  dot = CreateGraph(data['buses'], type_data, Digraph)

  dot.engine = 'dot'
  print(dot.source)
  dot.render('fcviz_out.png', view=True)

if __name__ == '__main__':
  main()
