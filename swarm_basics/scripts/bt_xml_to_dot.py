#!/usr/bin/env python3
"""bt_xml_to_dot.py — render a BehaviorTree.CPP v3 tree XML as a Graphviz diagram.

Usage:
    python3 bt_xml_to_dot.py tree.xml out.dot [--title "caption"]

Then render:
    dot -Tpng -Gdpi=200 out.dot -o out.png
    dot -Tpdf          out.dot -o out.pdf

Colour scheme (matches the thesis figures):
    green  : control-flow nodes (Sequence, Fallback, PipelineSequence, ...)
    orange : decorators (RateController, Timeout, KeepRunningUntilFailure, ...)
    blue   : action nodes (ComputePathToPose, FollowPath, Arc, ...)
    grey   : condition nodes (GoalUpdated, BlackboardCheck*, IsHumanClose, ...)
    Project-specific nodes keep the same four colours: the custom conditions
    (IsHumanClose, IsRobotClose, IsRearClear, CheckLlmAction) are drawn as
    conditions and the Arc action as an action — no fifth colour category.

Cosmetic conventions for the thesis figures:
    * BlackboardCheckBool / BlackboardCheckString are labelled "BlackboardCheck"
    * constant ports are hidden (FollowPath: path, controller_id;
      ComputePathToPose: path)

No ROS dependencies — pure xml.etree + text output, so it can run anywhere.
"""
import argparse
import xml.etree.ElementTree as ET

CONTROL = {
    "Sequence", "ReactiveSequence", "Fallback", "ReactiveFallback",
    "PipelineSequence", "RecoveryNode", "RoundRobin", "Parallel",
}
DECORATOR = {
    "RateController", "Timeout", "KeepRunningUntilFailure", "Inverter",
    "ForceSuccess", "ForceFailure", "Repeat", "RetryUntilSuccessful",
    "Delay", "DistanceController", "SpeedController",
}
CONDITION = {
    "GoalUpdated", "GlobalUpdatedGoal", "BlackboardCheckBool",
    "BlackboardCheckString", "IsStuck", "IsPathValid", "TransformAvailable",
    "GoalReached", "InitialPoseReceived", "TimeExpired",
    # project-specific condition nodes — same colour, no extra category
    "IsHumanClose", "IsRobotClose", "IsRearClear", "CheckLlmAction",
}
# cosmetic conventions for the thesis figures
LABEL_ALIAS = {
    "BlackboardCheckBool": "BlackboardCheck",
    "BlackboardCheckString": "BlackboardCheck",
}
HIDE_PORTS = {
    "FollowPath": {"path", "controller_id"},
    "ComputePathToPose": {"path"},
}

FILL = {
    "control":   "#c9e7c9",
    "decorator": "#ffd9a6",
    "action":    "#cfe2ff",
    "condition": "#e2e2e2",
}


def classify(tag):
    if tag in CONTROL:
        return "control"
    if tag in DECORATOR:
        return "decorator"
    if tag in CONDITION:
        return "condition"
    return "action"


def esc(s):
    return (str(s).replace("\\", "\\\\").replace('"', '\\"')
            .replace("{", "\\{").replace("}", "\\}")
            .replace("<", "\\<").replace(">", "\\>"))


def node_label(el):
    tag = el.tag
    disp = LABEL_ALIAS.get(tag, tag)
    name = el.get("name")
    lines = []
    if name and name != tag:
        lines.append(f"<B>{esc(name)}</B>")
    else:
        lines.append(f"<B>{esc(disp)}</B>")
    if name and name != tag:
        lines.append(f"<I>{esc(disp)}</I>")
    hide = HIDE_PORTS.get(tag, set())
    ports = [(k, v) for k, v in el.attrib.items()
             if k != "name" and k not in hide]
    if ports:
        # wrap only BETWEEN "k=v" pairs, never inside one, so long service
        # names stay readable and greppable in the generated figure
        pairs = [f"{k}={v}" for k, v in ports]
        packed, line = [], ""
        for p in pairs:
            cand = p if not line else line + ", " + p
            if len(cand) > 44 and line:
                packed.append(line)
                line = p
            else:
                line = cand
        if line:
            packed.append(line)
        for ln in packed:
            lines.append(f"<FONT POINT-SIZE=\"9\">{esc(ln)}</FONT>")
    return "<" + "<BR/>".join(lines) + ">"


def walk(el, dot, counter):
    """Emit node + children, return this node's id."""
    counter[0] += 1
    nid = f"n{counter[0]}"
    kind = classify(el.tag)
    dot.append(
        f'  {nid} [label={node_label(el)}, fillcolor="{FILL[kind]}"];\n'
    )
    for child in el:
        cid = walk(child, dot, counter)
        dot.append(f"  {nid} -> {cid};\n")
    return nid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xml")
    ap.add_argument("dot")
    ap.add_argument("--title", default=None)
    args = ap.parse_args()

    root = ET.parse(args.xml).getroot()
    tree = root.find("BehaviorTree") if root.tag == "root" else root

    dot = ["digraph BT {\n",
           '  graph [rankdir=TB, fontname="Helvetica", fontsize=11, '
           'labelloc="t", labeljust="l", nodesep=0.28, ranksep=0.38',
           f', label="{esc(args.title)}"' if args.title else "",
           '];\n',
           '  node [shape=box, style="rounded,filled", fontname="Helvetica", '
           'fontsize=10, margin="0.12,0.07"];\n',
           '  edge [color="#666666", arrowsize=0.7];\n']
    children = list(tree)
    if len(children) == 1:
        walk(children[0], dot, [0])
    else:  # multiple roots: join them under an invisible point
        dot.append('  root [shape=point, width=0.05, fillcolor="#000000"];\n')
        for child in children:
            cid = walk(child, dot, [0])
            dot.append(f"  root -> {cid};\n")
    dot.append("}\n")

    with open(args.dot, "w") as f:
        f.write("".join(dot))
    print(f"wrote {args.dot}")


if __name__ == "__main__":
    main()
