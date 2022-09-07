#!/usr/bin/env python3

import click
import json
from textwrap import indent
import os
import math
import itertools
from prettytable import PrettyTable

VARIANTS = list(itertools.product(["none", "naive", "online"], ["no", "coll", "yes"]))

def getSet(task):
    sets = {
        "tangled_1k": "experiments/snake_reconfig_ik/tangled_1k",
        "tangled_6k": "experiments/snake_reconfig_ik/tangled_6k",
        "tangled_m10": "experiments/snake_reconfig/tangled_m10",
        "matej": "data/configurations/snakeBench",
        "hand_crafted": "data/configurations/rofibots"
    }
    for name, path in sets.items():
        if path in task["command"]:
            return name
    raise RuntimeError(f"Unknown set for task {task['command']}")

def splitTasksToSets(tasks):
    res = {}
    for t in tasks:
        s = getSet(t)
        x = res.get(s, [])
        x.append(t)
        res[s] = x
    return res

def getCollisionType(task):
    types = ["none", "naive", "online"]
    for t in types:
        if f"--collisions={t}" in task["command"]:
            return t
    raise RuntimeError(f"Unknown collision for task {task}")


def getStraightening(task):
    types = ["no", "coll", "yes"]
    for t in types:
        if f"--straighten={t}" in task["command"]:
            return t
    raise RuntimeError(f"Unknown straightening for task {task}")

def countTasks(tasks):
    res = {}
    for v in VARIANTS:
        res[v] = {"succ": 0, "fail": 0, "limit": 0, "crash": 0, "time": 0}
    for t in tasks:
        cat = (getCollisionType(t), getStraightening(t))
        x = res[cat]
        if t["exitcode"] == 0:
            if t["result"]["result"]:
                x["succ"] += 1
                x["time"] += t["result"]["time"]
            else:
                x["fail"] += 1
        else:
            if t["stats"]["outOfMemory"] or t["stats"]["timeout"]:
                x["limit"] += 1
            else:
                x["crash"] += 1
        res[cat] = x
    return res

def taskModuleCount(task) -> int:
    filename = task["command"].split(" ")[-1]
    with open(filename) as f:
        return sum(1 for line in f.readlines() if line.startswith("M"))

def computeAvgTime(res, c):
    return res[c]['time'] // res[c]['succ'] / 1000 if res[c]['succ'] != 0 else "-"

def printResults(sets, taskFilter):
    table = PrettyTable()
    table.field_names = ["Test set"] + [f"{c}-{s}" for c, s in VARIANTS]
    for name, setTasks in sets.items():
        filteredTasks = [t for t in setTasks if taskFilter(t)]
        res = countTasks(filteredTasks)
        table.add_row([f"{name} ({len(filteredTasks)})"] + [f"{res[c]['succ']}/{res[c]['fail']}/{res[c]['limit']}/{res[c]['crash']}" for c in VARIANTS])
    print("Numbers represent: number of solved/number of unsolved/number of timeout/number of crashes")
    print("Types of collision: " + ",".join(["none", "naive", "online"]))
    print("Types of straightening: " + ",".join(["no", "coll", "yes"]))
    print(table)

    table = PrettyTable()
    table.field_names = ["Test set"] + [f"{c}-{s}" for c, s in VARIANTS]
    for name, filteredTasks in sets.items():
        filteredTasks = [t for t in filteredTasks if taskFilter(t)]
        res = countTasks(filteredTasks)
        table.add_row([name] + [f"{computeAvgTime(res, c)}" for c in VARIANTS])
    print("\nNumbers represent average solving time in seconds")
    print(table)


@click.command()
@click.argument("source", type=click.Path(exists=True, file_okay=True, dir_okay=False), nargs=-1)
def basestats(source):
    tasks = []
    for s in source:
        with open(s) as f:
            data = json.load(f)
            tasks += data["tasks"]
    sets = splitTasksToSets(tasks)
    print("\n=== Summary ===========================================================\n")
    printResults(sets, lambda t: True)
    for i in range(0, 100, 10):
        print(f"\n=== {i}-{i+10} ===========================================================\n")
        printResults(sets, lambda t: i < taskModuleCount(t) <= i + 10)

    print(f"\n=== Exactly 10 =========================================================\n")
    printResults(sets, lambda t: taskModuleCount(t) == 10)


@click.group()
def cli():
    pass

cli.add_command(basestats)

if __name__ == "__main__":
    cli()
