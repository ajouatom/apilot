import os
from tools.lib.url_file import URLFile

DATA_ENDPOINT = os.getenv("DATA_ENDPOINT", "http://data-raw.comma.internal/")

def FileReader(fn, debug=False):
  if fn.startswith("cd:/"):
    fn = fn.replace("cd:/", DATA_ENDPOINT)
  if fn.startswith("http://") or fn.startswith("https://"):
    return URLFile(fn, debug=debug)
  return open(fn, "rb")
