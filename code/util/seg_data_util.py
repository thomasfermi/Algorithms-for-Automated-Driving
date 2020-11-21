import requests
from tqdm import tqdm
import zipfile
import os
import shutil
from pathlib import Path

SEG_DATA_FOLDER = "data_lane_segmentation"

# Function from https://stackoverflow.com/a/62113293/2609987
def download(url: str, fname: str):
    resp = requests.get(url, stream=True)
    total = int(resp.headers.get("content-length", 0))
    with open(fname, "wb") as file, tqdm(
        desc=fname, total=total, unit="iB", unit_scale=True, unit_divisor=1024,
    ) as bar:
        for data in resp.iter_content(chunk_size=1024):
            size = file.write(data)
            bar.update(size)


def download_fresh_segmentation_data():
    os.makedirs(SEG_DATA_FOLDER)
    URL_data = "https://onedrive.live.com/download?cid=C6E4EE0D11B5A7CA&resid=C6E4EE0D11B5A7CA%21192&authkey=AJuKftLgRyb2PQo"
    download(URL_data, "temp_data_lane_segmentation.zip")
    print("Unzipping... You might want to go grab a coffee")
    with zipfile.ZipFile("temp_data_lane_segmentation.zip", "r") as zip_ref:
        zip_ref.extractall(SEG_DATA_FOLDER)
    os.remove("temp_data_lane_segmentation.zip")
    print("Done")


def download_segmentation_data():
    if os.path.exists(SEG_DATA_FOLDER):
        print(
            "You already have a folder 'data_lane_segmentation'. No download necessary."
        )
        print(
            "If you want to enforce the download, delete the folder 'data_lane_segmentation' and run this cell again."
        )
    else:
        download_fresh_segmentation_data()


def mkdir_if_not_exist(path):
    if not os.path.exists(path):
        os.makedirs(path)


def sort_collected_data():
    """ Copy and sort content of 'data' folder into 'data_lane_segmentation' folder  """

    def is_from_valid_set(fn):
        return fn.find("validation") != -1

    source_dir = "data"

    x_train_dir = os.path.join(SEG_DATA_FOLDER, "train")
    y_train_dir = os.path.join(SEG_DATA_FOLDER, "train_label")
    x_valid_dir = os.path.join(SEG_DATA_FOLDER, "val")
    y_valid_dir = os.path.join(SEG_DATA_FOLDER, "val_label")

    for direc in [x_train_dir, y_train_dir, x_valid_dir, y_valid_dir]:
        mkdir_if_not_exist(direc)

    images = [x for x in os.listdir(source_dir) if x.find("png") >= 0]
    inputs = [x for x in images if x.find("label") == -1]
    labels = [x for x in images if x.find("label") != -1]

    train_x = [x for x in inputs if not is_from_valid_set(x)]
    valid_x = [x for x in inputs if is_from_valid_set(x)]
    train_y = [x for x in labels if not is_from_valid_set(x)]
    valid_y = [x for x in labels if is_from_valid_set(x)]

    for f in train_x:
        shutil.copyfile(os.path.join("data", f), os.path.join(x_train_dir, f))

    for f in train_y:
        shutil.copyfile(os.path.join("data", f), os.path.join(y_train_dir, f))

    for f in valid_x:
        shutil.copyfile(os.path.join("data", f), os.path.join(x_valid_dir, f))

    for f in valid_y:
        shutil.copyfile(os.path.join("data", f), os.path.join(y_valid_dir, f))
