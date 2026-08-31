"""命令行入口：『绕湖走一圈』端到端演示。

用法：
  python run_demo.py                          # 默认任务：绕湖走一圈（当前坐标见 config.py）
  python run_demo.py --task "骑自行车绕湖一圈"
  python run_demo.py --lat 31.5163 --lng 120.2673 --name 蠡湖
  python run_demo.py --no-llm                 # 纯几何模式（不调用大模型）
  python run_demo.py --provider amap          # 用高德底图
"""
import argparse
import json
import os
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
os.environ.setdefault("PYTHONIOENCODING", "utf-8")

import config
from agent import run as agent_run
from llm import LLM


def main():
    ap = argparse.ArgumentParser(description="瓦片地图 + LangGraph 绕湖规划")
    ap.add_argument("--task", default="绕湖走一圈", help="任务描述")
    ap.add_argument("--lat", type=float, default=config.DEFAULT_CENTER[0])
    ap.add_argument("--lng", type=float, default=config.DEFAULT_CENTER[1])
    ap.add_argument("--name", default=config.DEFAULT_LOCATION_NAME)
    ap.add_argument("--provider", default=config.DEFAULT_PROVIDER,
                    choices=list(config.PROVIDERS))
    ap.add_argument("--no-llm", action="store_true", help="禁用大模型，纯几何规划")
    args = ap.parse_args()

    class _NoLLM:
        """占位：available=False 时所有节点走规则引擎。"""
        available = False

    llm = _NoLLM()
    if not args.no_llm:
        llm = LLM()
        if not llm.available:
            print("[warn] 未找到 DEEPSEEK_API_KEY，自动降级为纯几何模式")
        else:
            print(f"[info] LLM: text={llm.text_model} vision={llm.vision_model}")
    print(f"[info] 中心: {args.name} ({args.lat}, {args.lng}) | 底图: {args.provider}")
    print(f"[info] 任务: {args.task}")
    print("-" * 60)

    final = agent_run(task=args.task, center=(args.lat, args.lng),
                      provider=args.provider, llm=llm)

    print("-" * 60)
    print("== 思维链（LangGraph 节点轨迹）==")
    for t in final.get("trace", []):
        print(f"[{t['node']:11s}] {t['thought']}")
        if t.get("reasoning"):
            r = t["reasoning"].replace("\n", " ")
            print(f"             ↳ 思维链: {r[:180]}{'…' if len(r) > 180 else ''}")
    print("-" * 60)
    print("== 结果 ==")
    print(final.get("summary", ""))
    if final.get("run_dir"):
        print("输出目录:", final["run_dir"])
    print("状态:", final.get("status"))
    return 0 if final.get("status") == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
