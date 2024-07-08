import http.client
import os
import requests
import shutil
import socket
import urllib.error
import urllib.request

from openpilot.common.basedir import BASEDIR
from openpilot.system.version import get_build_metadata

from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_functions import MODELS_PATH, is_url_pingable

VERSION = "v3" if get_build_metadata().channel == "FrogPilot" else "v4"

GITHUB_REPOSITORY_URL = "https://raw.githubusercontent.com/FrogAi/FrogPilot-Resources/"
GITLAB_REPOSITORY_URL = "https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/"

DEFAULT_MODEL = "north-dakota-v2"
DEFAULT_MODEL_NAME = "North Dakota V2 (Default)"

NAVIGATION_MODELS = {"certified-herbalist", "duck-amigo", "los-angeles", "recertified-herbalist"}
RADARLESS_MODELS = {"radical-turtle"}

def get_repository_url():
  if is_url_pingable("https://github.com"):
    return GITHUB_REPOSITORY_URL

  if is_url_pingable("https://gitlab.com"):
    return GITLAB_REPOSITORY_URL

  return None

def delete_file(file):
  if os.path.exists(file):
    os.remove(file)

def download_file(destination, url, params_memory):
  try:
    with requests.get(url, stream=True, timeout=10) as r:
      r.raise_for_status()
      total_size = get_remote_file_size(url)
      downloaded_size = 0

      with open(destination, 'wb') as f:
        for chunk in r.iter_content(chunk_size=8192):
          if chunk:
            f.write(chunk)
            downloaded_size += len(chunk)
            progress = (downloaded_size / total_size) * 100
            if progress != 100:
              params_memory.put("ModelDownloadProgress", f"{progress:.0f}%")
            else:
              params_memory.put("ModelDownloadProgress", "Verifying authenticity...")

  except requests.HTTPError as http_error:
    status_code = http_error.response.status_code
    error_message = f"Failed: Server error ({status_code})"
    print(f"HTTP error occurred: {http_error} (Status code: {status_code})")
    params_memory.put("ModelDownloadProgress", error_message)
    delete_file(destination)

  except requests.ConnectionError as connection_error:
    error_message = "Failed: Connection dropped..."
    print(f"Connection error occurred: {connection_error}")
    params_memory.put("ModelDownloadProgress", error_message)
    delete_file(destination)

  except requests.Timeout as timeout_error:
    error_message = "Failed: Download timed out..."
    print(f"Timeout error occurred: {timeout_error}")
    params_memory.put("ModelDownloadProgress", error_message)
    delete_file(destination)

  except requests.RequestException as request_error:
    error_message = "Failed: Network request error. Check connection."
    print(f"Request error occurred: {request_error}")
    params_memory.put("ModelDownloadProgress", error_message)
    delete_file(destination)

  except Exception as e:
    error_message = "Failed: Unexpected error."
    print(f"An unexpected error occurred: {e}")
    params_memory.put("ModelDownloadProgress", error_message)
    delete_file(destination)

def get_remote_file_size(url):
  try:
    response = requests.head(url, timeout=10)
    response.raise_for_status()
    return int(response.headers.get('Content-Length', 0))

  except requests.RequestException as request_error:
    print(f"Error fetching file size: {request_error}")
    return None

def verify_download(file_path, model_url):
  if not os.path.exists(file_path):
    return False

  remote_file_size = get_remote_file_size(model_url)

  if remote_file_size is None:
    return False

  local_file_size = os.path.getsize(file_path)
  return remote_file_size == local_file_size

def download_model(params_memory):
  model = params_memory.get("ModelToDownload", encoding='utf-8')
  model_path = os.path.join(MODELS_PATH, f"{model}.thneed")

  if os.path.exists(model_path):
    error_message = f"Model {model} already exists, skipping download..."
    print(error_message)
    params_memory.put("ModelDownloadProgress", "Model already exists...")
    params_memory.remove("ModelToDownload")
    return

  repo_url = get_repository_url()
  if repo_url:
    model_url = f"{repo_url}Models/{model}.thneed"
    download_file(model_path, model_url, params_memory)

    if verify_download(model_path, model_url):
      success_message = f"Model {model} downloaded and verified successfully!"
      print(success_message)
      params_memory.put("ModelDownloadProgress", "Downloaded!")

    else:
      print(f"Model {model} verification failed. The file might be corrupted. Redownloading...")
      delete_file(model_path)
      download_file(model_path, model_url, params_memory)

      if verify_download(model_path, model_url):
        print(f"Model {model} redownloaded and verified successfully.")
      else:
        print(f"Model {model} redownload verification failed. The file might be corrupted.")

  else:
    error_message = "Github and Gitlab are offline..."
    print(error_message)
    params_memory.put("ModelDownloadProgress", error_message)

def update_models(params, params_memory, boot_run=True):
  base_url = get_repository_url()

  if base_url is None:
    print("Cannot determine base URL. Exiting...")
    return

  url = f"{base_url}Versions/model_names_{VERSION}.txt"

  try:
    with urllib.request.urlopen(url) as response:
      model_info = [line.decode('utf-8').strip().split(' - ') for line in response.readlines()]
      params.put("AvailableModels", ','.join(model[0] for model in model_info))
      params.put("AvailableModelsNames", ','.join(model[1] for model in model_info))
      print("Models list updated successfully.")

  except Exception as e:
    print(f"Failed to update models list. Error: {e}")
    return

  if not boot_run:
    return

  model_name = params.get("ModelName", encoding='utf-8')
  if "(Default)" in model_name and model_name != DEFAULT_MODEL_NAME:
    params.put("ModelName", model_name.replace(" (Default)", ""))

  available_models = params.get("AvailableModels", encoding='utf-8').split(',')
  for model_file in os.listdir(MODELS_PATH):
    if model_file.endswith('.thneed') and model_file[:-7] not in available_models:
      delete_file(os.path.join(MODELS_PATH, model_file))

  current_model = params.get("Model", encoding='utf-8')
  current_model_path = os.path.join(MODELS_PATH, f"{current_model}.thneed")
  if not os.path.exists(current_model_path):
    if current_model in available_models:
      params_memory.put("ModelToDownload", current_model)
    else:
      params.put("Model", DEFAULT_MODEL)
      params.put("ModelName", DEFAULT_MODEL_NAME)

  default_model_path = os.path.join(MODELS_PATH, f"{DEFAULT_MODEL}.thneed")
  if not os.path.exists(default_model_path):
    source_path = os.path.join(BASEDIR, "selfdrive/modeld/models/supercombo.thneed")
    if os.path.exists(source_path):
      shutil.copyfile(source_path, default_model_path)
      print(f"Copied default model from {source_path} to {default_model_path}")
    else:
      print(f"Source default model not found at {source_path}. Exiting...")
      return
