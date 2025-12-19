#!/usr/bin/env python3
import csv
from collections import defaultdict

def norm(s: str) -> str:
    return s.strip().lower()

def main():
    in_csv = "qlearning.csv"

    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        raise SystemExit(f"matplotlib not available: {e}")

    # Detect delimiter (your file uses ';')
    with open(in_csv, "r", newline="") as f:
        first = f.readline()
    delim = ";" if ";" in first else ","

    rows_by_env = defaultdict(list)

    with open(in_csv, newline="") as f:
        reader = csv.DictReader(f, delimiter=delim)

        # map normalized header -> real header
        header_map = {norm(h): h for h in (reader.fieldnames or [])}

        def pick(*cands):
            for c in cands:
                if c in header_map:
                    return header_map[c]
            return None

        col_env = pick("env", "environment", "world")
        col_ep  = pick("episode #", "episode", "episodes", "ep")
        col_rew = pick("cumulative reward", "cumulative_reward", "reward", "return")

        if not col_ep or not col_rew:
            raise SystemExit(f"Could not detect episode/reward columns. Header={reader.fieldnames}")

        for row in reader:
            env = row[col_env].strip() if col_env else "unknown"
            try:
                ep = float(row[col_ep])
                rw = float(row[col_rew])
            except Exception:
                continue
            rows_by_env[env].append((ep, rw))

    # Plot per environment
    for env, pts in rows_by_env.items():
        pts.sort(key=lambda x: x[0])
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        plt.figure()
        plt.plot(xs, ys)
        plt.xlabel("Episodes")
        plt.ylabel("Cumulative Reward")
        plt.tight_layout()
        out = f"qlearning_{env}.png"
        plt.savefig(out, dpi=180)
        plt.close()
        print("Wrote", out)

if __name__ == "__main__":
    main()
