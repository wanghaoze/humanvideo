"""Download a pinned upstream Quest APK and preserve provenance/license."""
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile

REV="9689484d319c4798e54d59509b192436647b7427"
REPO="jborbik/oculus_reader"
ROOT=Path(__file__).resolve().parents[1]
out=ROOT/"deliverables/quest3"
out.mkdir(parents=True,exist_ok=True)
url=f"https://media.githubusercontent.com/media/{REPO}/{REV}/oculus_reader/APK/teleop-debug.apk"
apk=out/"oculus-reader-quest3.apk"
if not apk.exists():
    print("Downloading",url,flush=True)
    urllib.request.urlretrieve(url,apk)
if not zipfile.is_zipfile(apk):
    raise RuntimeError("Download is not an APK/ZIP")
with zipfile.ZipFile(apk) as z:
    assert "AndroidManifest.xml" in z.namelist()
    assert any(x.startswith("lib/arm64-v8a/") for x in z.namelist())
license_url=f"https://raw.githubusercontent.com/{REPO}/{REV}/LICENSE"
(out/"LICENSE.oculus-reader").write_bytes(urllib.request.urlopen(license_url,timeout=30).read())
report={"upstream":f"https://github.com/{REPO}","revision":REV,"url":url,
        "apk":apk.name,"bytes":apk.stat().st_size,"sha256":hashlib.sha256(apk.read_bytes()).hexdigest(),
        "upstream_status":"Quest 3 beta; hardware validation required", "local_validation":"ZIP, manifest and arm64 library present"}
(out/"provenance.json").write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
