"""Separate synchronous Gazebo service client (no image callbacks / GIL lock).

gz-transport13's request_raw binding does not release the Python GIL. Keeping
it away from the subscriber process avoids starving its transport callbacks.
"""
import json
import sys


def main():
    from gz.transport13 import Node
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.msgs10.boolean_pb2 import Boolean
    node = Node()
    print(json.dumps({"ready": True}), flush=True)
    for line in sys.stdin:
        try:
            count = json.loads(line)["iterations"]
            if type(count) is not int or not 1 <= count <= 120:
                raise ValueError("Invalid finite step budget")
            req = WorldControl(pause=True, multi_step=count)
            ok, response = node.request("/world/toponav/control", req,
                                        WorldControl, Boolean, 15000)
            print(json.dumps({"ok": bool(ok and response.data)}), flush=True)
        except Exception as error:
            print(json.dumps({"ok": False, "error": str(error)}), flush=True)


if __name__ == "__main__":
    main()
