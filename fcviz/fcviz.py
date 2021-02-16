import sys
from graphviz import Digraph
import yaml
import IPython

def snake2camel(str):
  return ''.join(x.capitalize() or '_' for x in str.split('_'))

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

if len(sys.argv) != 2:
  print("Expecting fastcat configuration yaml file")
  sys.exit(0)

print("Attempting to open %s" % sys.argv[1])
data = yaml.load(open(sys.argv[1], 'r'), Loader=yaml.Loader)


type_data = yaml.load(open('src/fcgen/fastcat_types.yaml', 'r'), Loader=yaml.Loader)

bus_data = data['buses']

dot = Digraph(comment="fcviz")
dot.graph_attr['nodesep'] = '1'
dot.node_attr['shape'] = 'record'
dot.node_attr['width'] = '0.1'
dot.node_attr['height'] = '0.1'

dot_edges = [];

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
      for sig in dev['signals']:

        if sig['observed_device_name'] == 'FIXED_VALUE':
          cmder = dev['name']
          field = sig['cmd_field_name']
          val = str(sig['fixed_value'])
          from_node_str = cmder+'_'+field
          dot.node(from_node_str, label=val)
          #label_str = from_node_str + " = " + val
          label_str = ''
          to_node_str = dev['name'] + ':' + sig['cmd_field_name']
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

dot.engine = 'dot'
print(dot.source)
dot.render('fcviz_out.png', view=True)
