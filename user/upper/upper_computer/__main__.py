"""入口：python -m upper_computer

启动时自动重新生成 docs/PROTOCOL.md，保证文档与协议实现始终一致。
"""
import os
import sys


def _autogen_doc():
    try:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sys.path.insert(0, os.path.join(root, "tools"))
        import gen_protocol_doc as g
        out = g.OUT
        new = g.gen()
        old = ""
        if os.path.exists(out):
            with open(out, encoding="utf-8") as f:
                old = f.read()
        # 忽略生成时间行的差异
        def strip_ts(s):
            return "\n".join(l for l in s.splitlines() if not l.startswith("> 生成时间"))
        if strip_ts(old) != strip_ts(new):
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, "w", encoding="utf-8") as f:
                f.write(new)
            print(f"[doc] 协议文档已自动更新: {out}")
    except Exception as e:
        print(f"[doc] 文档生成跳过: {e}")


def main():
    _autogen_doc()
    from .ui import run
    run()


if __name__ == "__main__":
    main()
