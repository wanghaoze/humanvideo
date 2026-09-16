"""Package the fixed APK, source overlay and local diagnostic commands."""
from pathlib import Path
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parents[1]
files=['QUEST_READER_FIX.md','scripts/build_quest_reader.ps1','scripts/check_quest_reader.py',
       'scripts/install_quest_apk.ps1','r1pro_teleop/quest_bridge.py','tests/test_quest_tracking.py',
       'deliverables/quest3/humanvideo-quest-reader.apk','deliverables/quest3/fixed-provenance.json',
       'third_party/oculus_reader_fixed/LICENSE',
       'third_party/Meta-OpenXR-SDK-v85/LICENSE.txt',
       'third_party/Meta-OpenXR-SDK-v85/OPENXR_SDK_THIRD_PARTY_NOTICES.txt',
       'third_party/Meta-OpenXR-SDK-v85/Samples/SampleXrFramework/Src/Input/TinyUI.cpp']
files += ['third_party/oculus_reader_fixed/app/'+x for x in [
    'CMakeLists.txt','build.gradle','settings.gradle','AndroidManifest.xml','gradlew.bat',
    'gradle/wrapper/gradle-wrapper.properties','gradle/wrapper/gradle-wrapper.jar',
    'java/com/rail/oculus/teleop/MainActivity.java','res/values/strings.xml',
    'res/raw/efigs.fnt','res/raw/efigs_sdf.ktx','src/Remotes.h','src/fixed_main.cpp']]
out=root/'deliverables/quest3/humanvideo-quest-fix.zip'
manifest={}
with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
    for name in files:
        data=(root/name).read_bytes()
        z.writestr(name,data)
        manifest[name]=hashlib.sha256(data).hexdigest()
    z.writestr('SOURCE_OVERLAY.txt',
        'Clone Oculus Reader at 9689484d319c4798e54d59509b192436647b7427 into third_party/oculus_reader_fixed,\n'
        'and Meta-OpenXR-SDK at bbed2f20e38a5df7113630771c83cb8279e4fc26 into third_party/Meta-OpenXR-SDK-v85.\n'
        'Then overlay these files. This is a source overlay, not the full SDK or Android toolchain.\n')
    z.writestr('FILES_SHA256.json',json.dumps(manifest,indent=2))
with zipfile.ZipFile(out) as z:
    assert z.testzip() is None
    for name,digest in manifest.items():assert hashlib.sha256(z.read(name)).hexdigest()==digest
print(json.dumps({'path':str(out),'bytes':out.stat().st_size,'sha256':hashlib.sha256(out.read_bytes()).hexdigest()}))
