#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.8"
# dependencies = [
#   "graphviz",
#   "PyYAML",
# ]
# ///
# If you prefer not to use uv, install the dependencies listed above and run
# this file with python3 directly.

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any


def ParseArgs() -> argparse.Namespace:
    """Parse command-line arguments for the fcviz script.

    Returns
    -------
    argparse.Namespace
        Parsed command-line arguments containing the input YAML path.
    """
    parser = argparse.ArgumentParser(
        description="Generate a Graphviz diagram from a fastcat configuration YAML file."
    )
    parser.add_argument(
        "config_yaml",
        type=Path,
        help="Path to the fastcat configuration YAML file.",
    )
    return parser.parse_args()


def snake2camel(value: str) -> str:
    """Convert a snake_case string to CamelCase.

    Parameters
    ----------
    value : str
        Input string in snake_case.

    Returns
    -------
    str
        CamelCase version of ``value``.
    """
    return "".join(x.capitalize() or "_" for x in value.split("_"))


def GetCommandFields(type_data: dict[str, Any], input_command: str) -> list[str]:
    """Look up fields for a fastcat command definition.

    Parameters
    ----------
    type_data : dict[str, Any]
        Parsed contents of ``fastcat_types.yaml``.
    input_command : str
        Command name from the device configuration.

    Returns
    -------
    list[str]
        Field names defined for the matching command.
    """
    field_list: list[str] = []
    for command in type_data["commands"]:
        command_name = command["name"] + "_cmd"
        print(
            "checking command: %s against input: %s"
            % (command_name, input_command.lower())
        )
        if command_name == input_command.lower():
            print("matched %s, now appending fields..." % command_name)
            for field in command["fields"]:
                field_list.append(field["name"])

    return field_list


def GetStateFields(type_data: dict[str, Any], device_class_name: str) -> list[str]:
    """Look up state fields for a fastcat device class.

    Parameters
    ----------
    type_data : dict[str, Any]
        Parsed contents of ``fastcat_types.yaml``.
    device_class_name : str
        Device class name from the configuration.

    Returns
    -------
    list[str]
        Field names defined for the matching device state.
    """
    field_list: list[str] = []
    for state in type_data["states"]:
        camel = snake2camel(state["name"])
        print("checking state: %s against input: %s" % (camel, device_class_name))
        if camel == device_class_name:
            print("matched %s, now appending fields..." % state["name"])
            for field in state["fields"]:
                field_list.append(field["name"])

    return field_list


def CreateGraph(
    bus_data: list[dict[str, Any]], type_data: dict[str, Any], Digraph: Any
) -> Any:
    """Build the Graphviz representation of a fastcat bus configuration.

    Parameters
    ----------
    bus_data : list[dict[str, Any]]
        Bus definitions loaded from the fastcat configuration YAML file.
    type_data : dict[str, Any]
        Parsed contents of ``fastcat_types.yaml``.
    Digraph : Any
        Graphviz ``Digraph`` class used to construct the output graph.

    Returns
    -------
    Any
        Constructed Graphviz graph object.
    """
    dot = Digraph(comment="fcviz")
    dot.graph_attr["nodesep"] = "1"
    dot.node_attr["shape"] = "record"
    dot.node_attr["width"] = "0.1"
    dot.node_attr["height"] = "0.1"

    for bus in bus_data:
        for dev in bus["devices"]:
            ## Create a New node for the device
            if dev["device_class"] == "IGNORE":
                continue
            elif dev["device_class"] == "Commander":
                fields = GetCommandFields(type_data, dev["device_cmd_type"])
                node_label = (
                    dev["device_class"] + " | <" + dev["name"] + "> " + dev["name"]
                )
                node_label = node_label + " | <enable> enable"
                node_label = node_label + " | " + dev["device_cmd_type"].upper()
                for field in fields:
                    node_label = node_label + " | <" + field + "> " + field
                print(node_label)
                node_label = "{" + node_label + "}"
                print(node_label)
                dot.node(dev["name"], label=node_label)
            else:
                fields = GetStateFields(type_data, dev["device_class"])
                node_label = (
                    dev["device_class"] + " | <" + dev["name"] + "> " + dev["name"]
                )
                for field in fields:
                    node_label = node_label + " | <" + field + "> " + field
                print(node_label)
                node_label = "{" + node_label + "}"
                print(node_label)
                dot.node(dev["name"], label=node_label)

            ## Create edges for signals

            if dev["device_class"] == "Commander":
                dot.edge(
                    dev["name"], dev["device_cmd_name"], label=dev["device_cmd_type"]
                )

            if "signals" in dev:
                n_sig = 0
                for sig in dev["signals"]:
                    n_sig += 1

                    if sig["observed_device_name"] == "FIXED_VALUE":
                        cmder = dev["name"]
                        if "cmd_field_name" in sig:
                            field = sig["cmd_field_name"]
                        else:
                            field = str(n_sig)
                        val = str(sig["fixed_value"])
                        from_node_str = cmder + "_" + field
                        dot.node(from_node_str, label=val)
                        # label_str = from_node_str + " = " + val
                        label_str = ""
                        to_node_str = dev["name"] + ":" + field
                    elif dev["device_class"] == "Commander":
                        from_node_str = (
                            sig["observed_device_name"]
                            + ":"
                            + sig["request_signal_name"]
                        )
                        label_str = sig["request_signal_name"]
                        to_node_str = dev["name"] + ":" + sig["cmd_field_name"]
                    else:
                        from_node_str = (
                            sig["observed_device_name"]
                            + ":"
                            + sig["request_signal_name"]
                        )
                        label_str = sig["request_signal_name"]
                        to_node_str = dev["name"] + ":" + dev["name"]

                    print("from: %s; to: %s" % (from_node_str, to_node_str))
                    dot.edge(from_node_str, to_node_str, label=label_str)

    return dot


def main() -> None:
    """Load input data, generate the graph, and render the output image."""
    args = ParseArgs()

    from graphviz import Digraph  # type: ignore[import-not-found]
    import yaml  # type: ignore[import-untyped]

    print("Attempting to open %s" % args.config_yaml)
    with args.config_yaml.open("r") as config_file:
        data: dict[str, Any] = yaml.load(config_file, Loader=yaml.Loader)

    with Path("src/fcgen/fastcat_types.yaml").open("r") as types_file:
        type_data: dict[str, Any] = yaml.load(types_file, Loader=yaml.Loader)

    dot = CreateGraph(data["buses"], type_data, Digraph)

    dot.engine = "dot"
    print(dot.source)
    dot.render("fcviz_out.png", view=True)


if __name__ == "__main__":
    main()
