#!/usr/bin/env python3
import argparse
import json
import sys
from urllib import error, parse, request


def _http(method: str, url: str, payload: dict | None = None) -> dict:
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = request.Request(url, data=data, headers=headers, method=method)
    try:
        with request.urlopen(req, timeout=20) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        try:
            payload = json.loads(body)
        except json.JSONDecodeError:
            payload = {"error": body or exc.reason}
        raise SystemExit(f"HTTP {exc.code}: {json.dumps(payload, ensure_ascii=False)}") from exc


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CLI for the OpenClaw platoon bridge server")
    parser.add_argument("--base-url", default="http://host.docker.internal:18801")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("health")
    sub.add_parser("snapshot")

    platoon = sub.add_parser("platoon")
    platoon.add_argument("platoon_id")

    candidates = sub.add_parser("candidates")
    candidates.add_argument("platoon_id")

    transfer = sub.add_parser("transfer")
    transfer.add_argument("request_id")

    request_cmd = sub.add_parser("request")
    request_cmd.add_argument("vehicle_id")
    request_cmd.add_argument("from_platoon_id")
    request_cmd.add_argument("to_platoon_id")
    request_cmd.add_argument("--reason", default="destination_match")
    request_cmd.add_argument("--sender-agent")
    request_cmd.add_argument("--receiver-agent")

    accept = sub.add_parser("accept")
    accept.add_argument("request_id")
    accept.add_argument("--reason", default="accepted")
    accept.add_argument("--sender-agent", default="platoon_b")
    accept.add_argument("--receiver-agent", default="platoon_a")

    reject = sub.add_parser("reject")
    reject.add_argument("request_id")
    reject.add_argument("--reason", default="rejected")
    reject.add_argument("--sender-agent", default="platoon_b")
    reject.add_argument("--receiver-agent", default="platoon_a")

    commit = sub.add_parser("commit")
    commit.add_argument("request_id")

    return parser


def main() -> int:
    args = build_parser().parse_args()
    base = args.base_url.rstrip("/")

    if args.command == "health":
        payload = _http("GET", f"{base}/health")
    elif args.command == "snapshot":
        payload = _http("GET", f"{base}/snapshot")
    elif args.command == "platoon":
        payload = _http("GET", f"{base}/platoons/{parse.quote(args.platoon_id)}")
    elif args.command == "candidates":
        payload = _http("GET", f"{base}/platoons/{parse.quote(args.platoon_id)}/transfer-candidates")
    elif args.command == "transfer":
        payload = _http("GET", f"{base}/transfers/{parse.quote(args.request_id)}")
    elif args.command == "request":
        payload = _http(
            "POST",
            f"{base}/transfers",
            {
                "vehicle_id": args.vehicle_id,
                "from_platoon_id": args.from_platoon_id,
                "to_platoon_id": args.to_platoon_id,
                "reason": args.reason,
                "sender_agent": args.sender_agent or args.from_platoon_id,
                "receiver_agent": args.receiver_agent or args.to_platoon_id,
            },
        )
    elif args.command == "accept":
        payload = _http(
            "POST",
            f"{base}/transfers/{parse.quote(args.request_id)}/accept",
            {
                "reason": args.reason,
                "sender_agent": args.sender_agent,
                "receiver_agent": args.receiver_agent,
            },
        )
    elif args.command == "reject":
        payload = _http(
            "POST",
            f"{base}/transfers/{parse.quote(args.request_id)}/reject",
            {
                "reason": args.reason,
                "sender_agent": args.sender_agent,
                "receiver_agent": args.receiver_agent,
            },
        )
    elif args.command == "commit":
        payload = _http("POST", f"{base}/transfers/{parse.quote(args.request_id)}/commit", {})
    else:
        raise SystemExit(f"Unknown command: {args.command}")

    json.dump(payload, sys.stdout, indent=2, ensure_ascii=False)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
