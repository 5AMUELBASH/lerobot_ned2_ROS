#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def print_result(question: str, passed: bool, detail: str | None = None) -> None:
    status = "PASS" if passed else "FAIL"
    line = f"[{status}] {question}"
    if detail:
        line += f" - {detail}"
    print(line)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a minimal LeRobot dataset validity check and print PASS/FAIL results."
    )
    parser.add_argument(
        "--repo-id",
        type=str,
        required=True,
        help="Dataset repo id, e.g. 'lerobot/pusht'.",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=None,
        help="Optional local dataset root. If omitted, LeRobot will use cache/download behavior.",
    )
    parser.add_argument(
        "--expected-robot-type",
        type=str,
        required=True,
        help="Expected robot_type value to compare against dataset metadata.",
    )
    parser.add_argument(
        "--sample-index",
        type=int,
        default=0,
        help="Sample index to load for the sample-level checks.",
    )
    parser.add_argument(
        "--required-fields",
        nargs="+",
        default=["observation.state", "action"],
        help="Required sample keys that must be present in the loaded sample.",
    )
    args = parser.parse_args()

    dataset = None
    sample = None
    all_passed = True

    try:
        dataset = LeRobotDataset(args.repo_id, root=args.root)
        print_result(
            "Does the dataset construct and load without error?",
            True,
            f"root={dataset.root}",
        )
    except Exception as exc:  # noqa: BLE001
        all_passed = False
        print_result(
            "Does the dataset construct and load without error?",
            False,
            f"{type(exc).__name__}: {exc}",
        )

    if dataset is not None:
        actual_robot_type = dataset.meta.robot_type
        robot_type_ok = actual_robot_type == args.expected_robot_type
        all_passed &= robot_type_ok
        print_result(
            "Does the dataset report the expected robot_type?",
            robot_type_ok,
            f"expected={args.expected_robot_type}, actual={actual_robot_type}",
        )

        features = dataset.features
        feature_schema_ok = isinstance(features, dict) and len(features) > 0
        all_passed &= feature_schema_ok
        print_result(
            "Does the dataset expose a feature schema?",
            feature_schema_ok,
            f"{len(features) if isinstance(features, dict) else 0} features",
        )

        try:
            sample = dataset[args.sample_index]
            print_result(
                "Can an actual sample be loaded?",
                True,
                f"sample_index={args.sample_index}",
            )
        except Exception as exc:  # noqa: BLE001
            all_passed = False
            print_result(
                "Can an actual sample be loaded?",
                False,
                f"{type(exc).__name__}: {exc}",
            )
    else:
        print_result(
            "Does the dataset report the expected robot_type?",
            False,
            "dataset failed to construct",
        )
        print_result(
            "Does the dataset expose a feature schema?",
            False,
            "dataset failed to construct",
        )
        print_result(
            "Can an actual sample be loaded?",
            False,
            "dataset failed to construct",
        )

    if sample is not None:
        missing_fields = [field for field in args.required_fields if field not in sample]
        required_fields_ok = len(missing_fields) == 0
        all_passed &= required_fields_ok
        detail = f"missing={missing_fields}" if missing_fields else f"present={args.required_fields}"
        print_result(
            "Are required fields present in loaded samples?",
            required_fields_ok,
            detail,
        )
    else:
        all_passed = False
        print_result(
            "Are required fields present in loaded samples?",
            False,
            "sample could not be loaded",
        )

    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
