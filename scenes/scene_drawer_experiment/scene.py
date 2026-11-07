"""Native NNN scene entry for the drawer experiments in Mock mode."""
from drawer_agent.experiments import STAGES, mock
from drawer_agent.runner import FAULTS


def run(params):
    if params.get("backend", "mock") != "mock":
        return {"success": False, "feedback": "该场景入口只支持 Mock，真机使用实验运行单中的入口"}
    experiment = params.get("experiment", 1)
    if type(experiment) is not int or experiment not in STAGES:
        return {"success": False, "feedback": "实验编号必须是 1、2 或 3"}
    stages = STAGES[experiment]
    stage = params.get("stage", next(iter(stages)))
    faults = params.get("faults", [])
    if not isinstance(stage, str) or stage not in stages or not isinstance(faults, list) or any(
            not isinstance(fault, str) or fault not in FAULTS for fault in faults):
        return {"success": False, "feedback": "实验阶段或 Mock 故障参数无效"}
    result = mock(experiment, stage, faults=faults)
    return {"success": result["status"] == "ok",
            "feedback": f"Mock 实验 {experiment} {stage} {result['status']}",
            "details": result}
