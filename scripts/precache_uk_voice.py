#!/usr/bin/env python3
"""Pre-download Google UK gTTS voice clips (and optional offline Vosk STT model)
while connected to home Wi-Fi so the ProFab lab demo runs with zero internet.

Usage:
  python3 scripts/precache_uk_voice.py              # Cache Google UK MP3 clips only
  python3 scripts/precache_uk_voice.py --with-vosk  # Also download ~40 MB offline Vosk STT model
"""

import argparse
import hashlib
import json
import os
import shutil
import urllib.request
import zipfile

try:
    from gtts import gTTS
except ImportError:
    gTTS = None

VOSK_URL = "https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip"
VOSK_FOLDER = "vosk-model-small-en-us-0.15"


def tts_cache_key(text: str, lang: str = "en", tld: str = "co.uk") -> str:
    digest = hashlib.sha1(f"{lang}|{tld}|{text.strip().lower()}".encode("utf-8")).hexdigest()[:16]
    return f"{digest}.mp3"


def collect_phrases(catalog_path: str) -> list[str]:
    phrases = [
        "TrainerBot systems checked and ready. Choose an exercise or ask me to suggest a routine.",
        "Auto-framing follower activated.",
        "Auto-framing follower paused.",
        "Stopped safely. Take a breather whenever you need.",
        "Stopping right away as requested. You are safe.",
        "No exercise is active right now. Pick one on your phone or ask me to suggest a routine.",
        "No exercise is active right now. Select an exercise button first to hear form details.",
        "Offline routine suggestion: start with Left Elbow Flexion for 5 slow reps, then try Seated Shoulder Raise with your torso upright.",
        "I am listening in offline lab mode. Say left elbow, right elbow, shoulder raise, core reach, details, routine, or stop.",
    ]

    if os.path.exists(catalog_path):
        with open(catalog_path, "r", encoding="utf-8") as f:
            catalog = json.load(f)
        for ex in catalog.get("exercises", []):
            cues = ex.get("cues", {})
            warnings = ex.get("safety_warnings", {})
            faq = ex.get("faq", {})
            for key in ("start", "flexed", "completed"):
                if cues.get(key):
                    phrases.append(cues[key])
            for key in ("tilt_warning", "speed_warning"):
                if warnings.get(key):
                    phrases.append(warnings[key])
            target_reps = int(ex.get("target_reps", 5))
            rep_done = cues.get("rep_done", "")
            for r in range(1, target_reps):
                phrases.append(f"Rep {r} of {target_reps}. {rep_done}")
            detail_msg = f"{ex['name']}: {ex.get('goal', '')} {faq.get('form', '')}".strip()
            if detail_msg:
                phrases.append(detail_msg)

    # Deduplicate preserving order
    seen = set()
    unique = []
    for p in phrases:
        clean = p.strip()
        if clean and clean not in seen:
            seen.add(clean)
            unique.append(clean)
    return unique


def cache_gtts_phrases(phrases: list[str], cache_dir: str):
    if gTTS is None:
        print("[ERROR] `gTTS` not installed. Run: pip install --break-system-packages gTTS")
        return 1
    os.makedirs(cache_dir, exist_ok=True)
    saved = 0
    skipped = 0
    for text in phrases:
        out_path = os.path.join(cache_dir, tts_cache_key(text))
        if os.path.exists(out_path) and os.path.getsize(out_path) > 0:
            skipped += 1
            continue
        tts = gTTS(text=text, lang="en", tld="co.uk", slow=False)
        tts.save(out_path)
        saved += 1
        print(f"  [CACHED] {out_path} <- {text[:55]}...")
    print(f"[OK] Google UK TTS cache ready in {cache_dir} ({saved} downloaded, {skipped} already cached).")
    return 0


def cache_vosk_model():
    vosk_root = os.path.expanduser("~/.cache/trainerbot_vosk")
    model_dir = os.path.join(vosk_root, "model")
    if os.path.isdir(model_dir) and os.listdir(model_dir):
        print(f"[OK] Offline Vosk model already present at {model_dir}.")
        return 0

    os.makedirs(vosk_root, exist_ok=True)
    zip_path = os.path.join(vosk_root, f"{VOSK_FOLDER}.zip")
    print(f"Downloading offline Vosk STT model (~40 MB) from {VOSK_URL}...")
    urllib.request.urlretrieve(VOSK_URL, zip_path)
    with zipfile.ZipFile(zip_path, "r") as zf:
        zf.extractall(vosk_root)
    extracted = os.path.join(vosk_root, VOSK_FOLDER)
    if os.path.isdir(extracted):
        if os.path.isdir(model_dir):
            shutil.rmtree(model_dir)
        os.rename(extracted, model_dir)
    if os.path.exists(zip_path):
        os.remove(zip_path)
    print(f"[OK] Offline Vosk model extracted to {model_dir}.")
    try:
        import vosk  # noqa: F401
        print("[OK] Python `vosk` package is installed.")
    except ImportError:
        print("[INFO] Install Python `vosk` for offline voice commands:")
        print("       pip install --break-system-packages vosk")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Pre-cache Google UK voice clips and optional Vosk STT.")
    parser.add_argument("--with-vosk", action="store_true", help="Also download offline Vosk EN model.")
    args = parser.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    catalog_path = os.path.normpath(os.path.join(here, "..", "config", "exercise_catalog.json"))
    cache_dir = os.path.expanduser("~/.cache/trainerbot_tts")

    phrases = collect_phrases(catalog_path)
    rc = cache_gtts_phrases(phrases, cache_dir)
    if args.with_vosk:
        rc = max(rc, cache_vosk_model())
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
