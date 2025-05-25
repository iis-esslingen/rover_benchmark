#!/usr/bin/env python3

import argparse
import subprocess
import string
import os
import sys
import zipfile


DATES = {
    "campus_small": {
        "autumn": "2023-11-23",
        "winter": "2024-02-19",
        "spring": "2024-04-14",
        "summer": "2023-09-11",
        "day": "2024-05-07",
        "dusk": "2024-05-08_1",
        "night": "2024-05-08_2",
        "night-light": "2024-05-24_1",
    },
    "campus_large": {
        "autumn": "2023-11-07",
        "winter": "2024-01-27",
        "spring": "2024-04-14",
        "summer": "2023-07-20",
        "day": "2024-09-25",
        "dusk": "2024-09-24_2",
        "night": "2024-09-24_3",
        "night-light": "2024-09-24_4",
    },
    "garden_small": {
        "autumn": "2023-09-15",
        "winter": "2024-01-13",
        "spring": "2024-04-11",
        "summer": "2023-08-18",
        "day": "2024-05-29_1",
        "dusk": "2024-05-29_2",
        "night": "2024-05-29_3",
        "night-light": "2024-05-29_4",
    },
    "garden_large": {
        "autumn": "2023-12-21",
        "winter": "2024-01-13",
        "spring": "2024-04-11",
        "summer": "2023-08-18",
        "day": "2024-05-29_1",
        "dusk": "2024-05-29_2",
        "night": "2024-05-30_1",
        "night-light": "2024-05-30_2",
    },
    "park": {
        "autumn": "2023-11-07",
        "spring": "2024-04-14",
        "summer": "2023-07-31",
        "day": "2024-05-08",
        "dusk": "2024-05-13_1",
        "night": "2024-05-13_2",
        "night-light": "2024-05-24_2",
        # Note: no winter data for 'park'
    },
}


def suffixes():
    for letter in string.ascii_lowercase[:9]:  # a->i
        yield 'a' + letter


def reconstruct_zip(save_dir, prefix, downloaded_parts, keep_parts):
    full_zip = os.path.join(save_dir, prefix)
    parts_list = " ".join(downloaded_parts)

    subprocess.run(f"cat {parts_list} > {full_zip}", shell=True, check=True)
    print(f"    Reconstruction complete for {full_zip}.")

    print(f"    Verifying integrity of {full_zip}...")
    try:
        with zipfile.ZipFile(full_zip, 'r') as zf:
            bad_file = zf.testzip()

        if bad_file is None:
            print(f"    Verification successful: all entries OK in {full_zip}.")

            if not keep_parts:
                for part in downloaded_parts:
                    try:
                        os.remove(part)
                    except OSError as e:
                        print(f"    Warning: could not remove {part}: {e}")
                print("    Removed chunk part files to free space.")
        else:
            print(f"    Warning: corrupted entry '{bad_file}' in {full_zip}.", file=sys.stderr)

    except zipfile.BadZipFile:
        print(f"    Error: {full_zip} is not a valid zip file.", file=sys.stderr)


def download_and_process(location, scenario, date_code, save_dir, keep_parts):
    prefix = f"{location}_{scenario}_{date_code}.zip"
    base_url = (
        "https://huggingface.co/datasets/iis-esslingen/ROVER/resolve/main/"
        f"{prefix}.part-"
    )
    downloaded_parts = []
    for suf in suffixes():
        part_name = f"{prefix}.part-{suf}"
        dest = os.path.join(save_dir, part_name)
        print(f"Downloading {base_url}{suf} --> {dest}")

        ret = subprocess.run(["wget", "-c", "-O", dest, base_url + suf]).returncode
        if ret != 0:
            if os.path.exists(dest) and os.path.getsize(dest) == 0:
                os.remove(dest)
                print(f"    Removed empty file {dest}")

            print(f"    Stopping at suffix '{suf}' (wget exit code {ret}).")
            break

        downloaded_parts.append(dest)

    if not downloaded_parts:
        print(f"    No parts for {location} {scenario}; skipping.")
        return
    
    reconstruct_zip(
        save_dir=save_dir,
        prefix=prefix,
        downloaded_parts=downloaded_parts,
        keep_parts=keep_parts
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Download, reconstruct, and extract split ZIP parts from ROVER dataset."
    )
    parser.add_argument(
        "--locations",
        nargs='+',
        choices=list(DATES.keys()),
        default=list(DATES.keys()),
        help=(
            f"Dataset locations to download; one or more of {list(DATES.keys())}. "
            "Default is all locations."
        )
    )
    parser.add_argument(
        "--scenarios",
        nargs='+',
        choices=["autumn", "winter", "spring", "summer", "day", "dusk", "night", "night-light"],
        default=["autumn", "winter", "spring", "summer", "day", "dusk", "night", "night-light"],
        help=(
            "Scenarios to download; one or more of the choices. "
            "Default is all scenarios."
        )
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default='.',
        help="Directory in which to save files (default: current directory)."
    )
    parser.add_argument(
        "--keep-parts",
        action="store_true",
        help="If set, retain chunked part files after successful reconstruction."
    )

    return parser.parse_args()


def main():
    args = parse_args()
    
    os.makedirs(args.save_dir, exist_ok=True)

    for location in args.locations:
        for scenario in args.scenarios:
            date_code = DATES.get(location, {}).get(scenario)

            if not date_code:
                print(f"    Warning: no data for {location} {scenario}, skipping.", file=sys.stderr)
                continue

            download_and_process(
                location=location, 
                scenario=scenario, 
                date_code=date_code, 
                save_dir=args.save_dir,
                keep_parts=args.keep_parts
            )

if __name__ == "__main__":
    main()