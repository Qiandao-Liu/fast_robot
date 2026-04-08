#!/usr/bin/env bash

set -euo pipefail

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg is required but not installed." >&2
  exit 1
fi

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <video_dir> [more_dirs...]" >&2
  exit 1
fi

for input_dir in "$@"; do
  if [ ! -d "$input_dir" ]; then
    echo "Skipping missing directory: $input_dir" >&2
    continue
  fi

  while IFS= read -r -d '' src; do
    dir="$(dirname "$src")"
    name="$(basename "$src")"
    stem="${name%.*}"
    ext="${name##*.}"
    out="$dir/$stem.mp4"
    tmp="$dir/$stem.websafe.tmp.mp4"

    ext_lc="$(printf '%s' "$ext" | tr '[:upper:]' '[:lower:]')"

    if [ "$ext_lc" = "mp4" ] && { [ -f "$dir/$stem.mov" ] || [ -f "$dir/$stem.MOV" ]; }; then
      echo "Skipping $src because a MOV source with the same stem exists."
      continue
    fi

    echo "Converting $src -> $out"
    ffmpeg -nostdin -y -i "$src" \
      -c:v libx264 -preset medium -crf 23 \
      -pix_fmt yuv420p -profile:v high -level 4.0 \
      -movflags +faststart \
      -c:a aac -b:a 128k \
      "$tmp"

    mv "$tmp" "$out"
  done < <(find "$input_dir" -type f \( -iname '*.mov' -o -iname '*.mp4' \) -print0)
done
