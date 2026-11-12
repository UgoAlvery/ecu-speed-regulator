#!/usr/bin/env bash
# Purge all *.docx files from the full git history of this repo.
#
# HOW TO USE
#   1. Run this script. It clones a bare mirror, rewrites history, and STOPS.
#   2. Review the before/after summary printed at the end.
#   3. Push manually when satisfied:
#        cd /tmp/ecu_purge_work/ecu-speed-regulator.git
#        git push --force --all
#        git push --force --tags
#
# REQUIREMENTS
#   git-filter-repo — install with: pip install git-filter-repo
#   (or apt install git-filter-repo on Debian/Ubuntu 23.04+)
#
# SAFETY
#   - Works on a TEMPORARY CLONE. The original repo is never touched.
#   - Script exits before any push. The user pushes manually.

set -euo pipefail

REMOTE=$(git -C "$(dirname "$0")/.." remote get-url origin 2>/dev/null || true)
if [ -z "$REMOTE" ]; then
    echo "ERROR: cannot determine remote URL. Run from inside the repo." >&2
    exit 1
fi

WORKDIR=$(mktemp -d /tmp/ecu_purge_work.XXXXXX)
MIRROR="$WORKDIR/mirror.git"

echo "=== Before: cloning mirror ==="
git clone --mirror "$REMOTE" "$MIRROR"
BEFORE_SIZE=$(du -sh "$MIRROR" | cut -f1)
BEFORE_COMMITS=$(git -C "$MIRROR" rev-list --all --count)
echo "  size:    $BEFORE_SIZE"
echo "  commits: $BEFORE_COMMITS"
echo ""

DOCX_LIST=$(git -C "$MIRROR" log --all --name-only --diff-filter=A --pretty=format: \
            | grep -i '\.docx$' | sort -u || true)
if [ -z "$DOCX_LIST" ]; then
    echo "No *.docx files found in history. Nothing to do."
    rm -rf "$WORKDIR"
    exit 0
fi

echo "=== Files to purge ==="
echo "$DOCX_LIST"
echo ""

echo "=== Running git filter-repo ==="
git -C "$MIRROR" filter-repo --invert-paths \
    $(echo "$DOCX_LIST" | sed "s|^|--path |" | tr '\n' ' ')

echo ""
echo "=== After ==="
AFTER_SIZE=$(du -sh "$MIRROR" | cut -f1)
AFTER_COMMITS=$(git -C "$MIRROR" rev-list --all --count)
echo "  size:    $AFTER_SIZE  (was $BEFORE_SIZE)"
echo "  commits: $AFTER_COMMITS  (was $BEFORE_COMMITS)"
echo "  purged:  $(echo "$DOCX_LIST" | wc -l) path(s)"
echo ""
echo "=== Script finished WITHOUT pushing ==="
echo "Review the rewritten mirror at: $MIRROR"
echo ""
echo "To push (DESTRUCTIVE — rewrites public history):"
echo "  cd $MIRROR"
echo "  git push --force --all"
echo "  git push --force --tags"
echo ""
echo "All collaborators must run: git fetch --all && git reset --hard origin/main"
