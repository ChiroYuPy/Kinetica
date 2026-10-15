#!/usr/bin/env bash
# Builds the website: a landing page, the testbed and the games, all running in the browser.
#
# Usage: scripts/build-site.sh
# Output: target/site/. Try it: python3 -m http.server -d target/site 8000
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
site="$root/target/site"
cd "$root"

rm -rf "$site"
mkdir -p "$site"

scripts/build-web.sh kinetica-testbed "Testbed"
cp -r target/web/kinetica-testbed "$site/testbed"
for game in slingshot trials platformer snip fruits; do
    scripts/build-web.sh "$game" "$game"
    mkdir -p "$site/games"
    cp -r "target/web/$game" "$site/games/$game"
done

cp web/site.html "$site/index.html"
cp docs/kinetica.png "$site/kinetica.png"
echo "Built $site"
