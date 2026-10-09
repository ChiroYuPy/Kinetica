#!/usr/bin/env bash
# Builds a game (or the testbed) for the web, ready to serve.
#
# Usage: scripts/build-web.sh <crate> [title]
# Output: target/web/<crate>/ with index.html, <crate>.wasm and mq_js_bundle.js.
# Try it: python3 -m http.server -d target/web/<crate> 8000, then open http://localhost:8000.
set -euo pipefail

crate="${1:?usage: scripts/build-web.sh <crate> [title]}"
title="${2:-$crate}"
root="$(cd "$(dirname "$0")/.." && pwd)"
out="$root/target/web/$crate"

cd "$root"
cargo build --release --target wasm32-unknown-unknown -p "$crate"

# The JS loader, and the scripts of the plugins the crate uses, must match the versions in
# use: take them from those crates' sources.
read -r macroquad_dir plugins < <(cargo metadata --format-version 1 | python3 -I -c '
import json, os, sys
metadata = json.load(sys.stdin)
packages = {p["id"]: p for p in metadata["packages"]}
# Dependencies of the crate, through the resolved graph.
nodes = {node["id"]: node for node in metadata["resolve"]["nodes"]}
root = next(id for id, p in packages.items() if p["name"] == sys.argv[1])
seen, stack = set(), [root]
while stack:
    id = stack.pop()
    if id not in seen:
        seen.add(id)
        stack.extend(nodes[id]["dependencies"])
directory = lambda name: next(os.path.dirname(packages[id]["manifest_path"]) for id in seen if packages[id]["name"] == name)
# Plugins whose JavaScript is not part of mq_js_bundle.js.
plugins = [f"{directory(name)}/js/{name}.js" for name in ["quad-url"] if any(packages[id]["name"] == name for id in seen)]
print(directory("macroquad"), " ".join(plugins))
' "$crate")

mkdir -p "$out"
cp "target/wasm32-unknown-unknown/release/$crate.wasm" "$out/"
cp "$macroquad_dir/js/mq_js_bundle.js" "$out/"
tags=""
for plugin in $plugins; do
    cp "$plugin" "$out/"
    tags+="    <script src=\"$(basename "$plugin")\"></script>\n"
done
sed -e "s/{{TITLE}}/$title/" -e "s/{{WASM}}/$crate.wasm/" -e "s#{{PLUGINS}}#$tags#" web/index.html > "$out/index.html"
echo "Built $out"
