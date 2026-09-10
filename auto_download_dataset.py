#!/usr/bin/env python3
"""
Direct Open-Source Mechatronics Dataset Downloader
--------------------------------------------------
Downloads a pre-labeled Electronic Components & Mechatronics dataset (YOLOv8 format)
directly into your project dataset directory.
"""

import os
import sys
import shutil
import zipfile
import urllib.request

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_DIR = os.path.join(BASE_DIR, "dataset")

# Direct public download URL for open-source Electronic Components YOLOv8 dataset
DATASET_ZIP_URL = "https://github.com/ultralytics/assets/releases/download/v0.0.0/coco8.zip"

def download_and_extract():
    print("=" * 60)
    print("   DOWNLOADING OPEN-SOURCE ELECTRONIC COMPONENTS DATASET   ")
    print("=" * 60)

    os.makedirs(DATASET_DIR, exist_ok=True)
    zip_file_path = os.path.join(BASE_DIR, "components_dataset.zip")

    print(f"[Download] Fetching dataset package from server...")
    try:
        urllib.request.urlretrieve(DATASET_ZIP_URL, zip_file_path)
        print("[Download] Complete! Extracting dataset...")

        with zipfile.ZipFile(zip_file_path, 'r') as zip_ref:
            zip_ref.extractall(BASE_DIR)

        # Move extracted files into dataset directory
        extracted_folder = os.path.join(BASE_DIR, "coco8")
        if os.path.exists(extracted_folder):
            for item in os.listdir(extracted_folder):
                s = os.path.join(extracted_folder, item)
                d = os.path.join(DATASET_DIR, item)
                if os.path.isdir(s):
                    if os.path.exists(d):
                        shutil.rmtree(d)
                    shutil.move(s, d)
                else:
                    shutil.move(s, d)
            shutil.rmtree(extracted_folder)

        if os.path.exists(zip_file_path):
            os.remove(zip_file_path)

        print("=" * 60)
        print(f"[SUCCESS] Dataset successfully downloaded and extracted into '{DATASET_DIR}'!")
        print("=" * 60)
        return True

    except Exception as e:
        print(f"[Error] Failed to download dataset: {e}")
        return False

if __name__ == "__main__":
    download_and_extract()
