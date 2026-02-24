#!/bin/bash

MYPATH=$PWD
# old/for Arduino builds
# FULLIMG=${MYPATH}/out.img
# UPDIMG=${MYPATH}/build/RX_FSK.ino.bin

# new/for pio build
FULLIMG=${MYPATH}/.pio/build/ttgo-lora32/firmware-image.bin
UPDIMG=${MYPATH}/.pio/build/ttgo-lora32/firmware.bin

setup_git() {
  GITHUB_API_KEY=`cat ~/.github.api.key`
}
generate_website_index() {
  #local template="${MYPATH}/rdzsonde-web/download-template.html"
  local template="_download-template.html"
  local main_rows=$(mktemp)
  local dev_rows=$(mktemp)
  local legacy_section=$(mktemp)
  trap "rm -f '$main_rows' '$dev_rows' '$legacy_section'" EXIT

  # MAIN_ROWS: one <tbody> per version so version + changelog share same stripe
  local first=1
  for f in $(ls main/*-full.bin 2>/dev/null | sort -r); do
    local vers=$(basename "$f" -full.bin)
    local ts=$(git log -1 --format="%ad" --date=format:"%b %d %H:%M:%S %Y" -- "main/$vers-full.bin" 2>/dev/null)

    # Build changelog snippet from up to 3 lines:
    # join with "; " unless the previous line already ends with punctuation.
    local cl=""
    local prev_raw=""
    local i=0
    if [ -f "main/${vers}-changelog.txt" ]; then
      while IFS= read -r line && [ "$i" -lt 5 ]; do
        i=$((i + 1))
        # HTML-escape each line
        local esc
        esc=$(printf '%s' "$line" | sed 's/&/\&amp;/g; s/</\&lt;/g; s/>/\&gt;/g; s/\"/\&quot;/g')
        if [ -z "$cl" ]; then
          cl="$esc"
        else
          if printf '%s' "$prev_raw" | grep -qE '[.!?:;]$'; then
            cl="${cl} $esc"
          else
            cl="${cl}; $esc"
          fi
        fi
        prev_raw="$line"
      done < "main/${vers}-changelog.txt"
    fi

    local meta_text=""; [ -n "$ts" ] && meta_text="$ts"; [ -n "$cl" ] && meta_text="${meta_text:+$meta_text }$cl"
    {
      echo "                <tbody>"
      echo "                  <tr>"
      echo "                    <td><code>$vers</code></td>"
      echo "                    <td>"
      echo "                      <span class=\"badge badge-type-full\">Full image</span>"
      echo "                    </td>"
      echo "                    <td>"
      echo "                      <a href=\"main/$vers-full.bin\">$vers-full.bin</a>"
      echo "                    </td>"
      echo "                  </tr>"
      if [ -n "$meta_text" ]; then
        echo "                  <tr class=\"download-meta-row\">"
        echo "                    <td colspan=\"3\"><small class=\"download-meta\">$meta_text</small></td>"
        echo "                  </tr>"
      fi
      if [ "$first" = 1 ]; then
        if [ -f main/update.ino.bin ]; then
          echo "                  <tr>"
          echo "                    <td><code>$vers</code></td>"
          echo "                    <td>"
          echo "                      <span class=\"badge badge-type-update\">Code update</span>"
          echo "                    </td>"
          echo "                    <td>"
          echo "                      <a href=\"main/update.ino.bin\">update.ino.bin</a>"
          echo "                    </td>"
          echo "                  </tr>"
        fi
        if [ -f main/update.fs.bin ]; then
          echo "                  <tr>"
          echo "                    <td><code>$vers</code></td>"
          echo "                    <td>"
          echo "                      <span class=\"badge badge-type-update\">Filesystem update</span>"
          echo "                    </td>"
          echo "                    <td>"
          echo "                      <a href=\"main/update.fs.bin\">update.fs.bin</a>"
          echo "                    </td>"
          echo "                  </tr>"
        fi
        first=0
      fi
      echo "                </tbody>"
    } >> "$main_rows"
  done

  # DEV_ROWS: one <tbody> per version so version + changelog share same stripe
  first=1
  for f in $(ls dev2/*-full.bin 2>/dev/null | sort -r); do
    vers=$(basename "$f" -full.bin)
    ts=$(git log -1 --format="%ad" --date=format:"%b %d %H:%M:%S %Y" -- "dev2/$vers-full.bin" 2>/dev/null)

    # Same changelog joining rules for dev builds
    cl=""
    prev_raw=""
    i=0
    if [ -f "dev2/${vers}-changelog.txt" ]; then
      while IFS= read -r line && [ "$i" -lt 3 ]; do
        i=$((i + 1))
        esc=$(printf '%s' "$line" | sed 's/&/\&amp;/g; s/</\&lt;/g; s/>/\&gt;/g; s/\"/\&quot;/g')
        if [ -z "$cl" ]; then
          cl="$esc"
        else
          if printf '%s' "$prev_raw" | grep -qE '[.!?:;]$'; then
            cl="${cl} $esc"
          else
            cl="${cl}; $esc"
          fi
        fi
        prev_raw="$line"
      done < "dev2/${vers}-changelog.txt"
    fi

    meta_text=""; [ -n "$ts" ] && meta_text="$ts"; [ -n "$cl" ] && meta_text="${meta_text:+$meta_text }$cl"
    {
      echo "                <tbody>"
      echo "                  <tr>"
      echo "                    <td><code>$vers</code></td>"
      echo "                    <td>"
      echo "                      <span class=\"badge badge-type-full\">Full image</span>"
      echo "                    </td>"
      echo "                    <td>"
      echo "                      <a href=\"dev2/$vers-full.bin\">$vers-full.bin</a>"
      echo "                    </td>"
      echo "                  </tr>"
      if [ -n "$meta_text" ]; then
        echo "                  <tr class=\"download-meta-row\">"
        echo "                    <td colspan=\"3\"><small class=\"download-meta\">$meta_text</small></td>"
        echo "                  </tr>"
      fi
      if [ "$first" = 1 ]; then
        if [ -f dev2/update.ino.bin ]; then
          echo "                  <tr>"
          echo "                    <td><code>$vers</code></td>"
          echo "                    <td>"
          echo "                      <span class=\"badge badge-type-update\">Code update</span>"
          echo "                    </td>"
          echo "                    <td>"
          echo "                      <a href=\"dev2/update.ino.bin\">update.ino.bin</a>"
          echo "                    </td>"
          echo "                  </tr>"
        fi
        if [ -f dev2/update.fs.bin ]; then
          echo "                  <tr>"
          echo "                    <td><code>$vers</code></td>"
          echo "                    <td>"
          echo "                      <span class=\"badge badge-type-update\">Filesystem update</span>"
          echo "                    </td>"
          echo "                    <td>"
          echo "                      <a href=\"dev2/update.fs.bin\">update.fs.bin</a>"
          echo "                    </td>"
          echo "                  </tr>"
        fi
        first=0
      fi
      echo "                </tbody>"
    } >> "$dev_rows"
  done

  # LEGACY_SECTION: latest from master and devel only, with EOL note
  {
    echo "        <section class=\"section\">"
    echo "          <div class=\"section-header\">"
    echo "            <div>"
    echo "              <h2 class=\"section-title\">Legacy builds</h2>"
    echo "              <p class=\"section-lead\">"
    echo "                Legacy builds for the old IDF environment. These are end-of-life and no longer updated; please use stable or development builds above for new deployments."
    echo "              </p>"
    echo "            </div>"
    echo "          </div>"
    echo "          <div class=\"table-scroll\">"
    echo "            <div class=\"table-scroll-inner\">"
    echo "              <table class=\"data-table\">"
    echo "                <thead>"
    echo "                  <tr>"
    echo "                    <th>Version</th>"
    echo "                    <th>Type</th>"
    echo "                    <th>Download</th>"
    echo "                  </tr>"
    echo "                </thead>"
    for dir in master devel; do
      [ ! -d "$dir" ] && continue
      latest_path=$(ls "$dir"/*-full.bin 2>/dev/null | sort -r | head -1)
      [ -z "$latest_path" ] && continue
      latest=$(basename "$latest_path")
      vers=$(echo "$latest" | sed 's/-full\.bin$//')
      ts=$(git log -1 --format="%ad" --date=format:"%b %d %H:%M:%S %Y" -- "$latest_path" 2>/dev/null)
      meta=""; [ -n "$ts" ] && meta="<br><small class=\"download-meta\">$ts</small>"
      echo "                <tbody>"
      echo "                  <tr>"
      echo "                    <td><code>$vers</code>$meta</td>"
      echo "                    <td><span class=\"badge badge-type-full\">Full image</span></td>"
      echo "                    <td><a href=\"$dir/$latest\">$latest</a></td>"
      echo "                  </tr>"
      echo "                </tbody>"
    done
    echo "              </table>"
    echo "            </div>"
    echo "          </div>"
    echo "        </section>"
  } >> "$legacy_section"

  # Merge template + generated content into download.html
  sed -e "/%%MAIN_ROWS%%/r $main_rows" -e "/%%MAIN_ROWS%%/d" \
      -e "/%%DEV_ROWS%%/r $dev_rows" -e "/%%DEV_ROWS%%/d" \
      -e "/%%LEGACY_SECTION%%/r $legacy_section" -e "/%%LEGACY_SECTION%%/d" \
      "$template" > download.html

  git add download.html
  git commit --amend --message "Build @ $(date)"
}
update_json_file() {
    local json_file="$1"
    local full_filename="$2"

    # Extract the version string from the filename (assuming pattern "devYYYYMMDD")
    local version=$(basename "$full_filename" | grep -oE 'dev[0-9]{8}')

    if [[ -z "$version" ]]; then
        echo "Error: Could not extract version from filename."
        return 1
    fi

    # Update the JSON file using jq
    jq --arg version "$version" --arg full_filename "$full_filename" '
        .version = $version |
        .builds[0].fwversion = $version |
        .builds[0].parts[0].path = $full_filename
    ' "$json_file" > "${json_file}.tmp" && mv "${json_file}.tmp" "$json_file"
}
commit_website_files() {
  BRANCH=`git branch --show-current`
  VERSION=`cat RX_FSK/version.h | grep version_id | egrep -o '".*"' | sed 's/"//g' | sed 's/ /_/g'`
  FSMAJOR=`cat RX_FSK/version.h | grep FS_MAJOR | perl -e '$_=<>;print /=(.*);/?chr($1+64):""'`
  FSMINOR=`cat RX_FSK/version.h | grep FS_MINOR | perl -e '$_=<>;print /=(.*);/?$1:""'`
  VERSION=$VERSION-$FSMAJOR$FSMINOR
  COMMIT_MESSAGE=`git log -1 --pretty=%B`

  MYPATH=$PWD
  echo "On branch $BRANCH"
  echo "Version $VERSION"
  cd /tmp
  git clone https://${GITHUB_API_KEY}@github.com/dl9rdz/rdz_ttgo_sonde.git -b gh-pages
  cd rdz_ttgo_sonde
  git config user.email "dl9rdz@darc.de"
  git config user.name "dl9rdz (via automated build)"
  git pull
  mkdir -p master
  mkdir -p devel
  mkdir -p main
  mkdir -p dev2
  cp ${FULLIMG} ${BRANCH}/${VERSION}-full.bin
  git add ${BRANCH}/${VERSION}-full.bin
  cp ${UPDIMG} ${BRANCH}/update.ino.bin
  git add ${BRANCH}/update.ino.bin
  echo "${COMMIT_MESSAGE}" >> ${BRANCH}/${VERSION}-changelog.txt
  git add ${BRANCH}/${VERSION}-changelog.txt
  echo "<html><body><p>${VERSION}</p></body></html>" > ${BRANCH}/update-info.html
  git add ${BRANCH}/update-info.html
  ${MYPATH}/scripts/makefsupdate.py ${MYPATH}/RX_FSK/data/ > ${BRANCH}/update.fs.bin
  git add ${BRANCH}/update.fs.bin
  update_json_file "manifest.json" "${BRANCH}/${VERSION}-full.bin"
  git add "manifest.json"
  git commit --message "Build @ `date`"
}
upload_files() {
  git push
}

# Optional: --nopush skips "git push" (for local testing)
NOPUSH=
for arg in "$@"; do
  [ "$arg" = "--nopush" ] && NOPUSH=1
done

setup_git
commit_website_files
generate_website_index
if [ -z "$NOPUSH" ]; then
  upload_files
else
  echo "Skipping git push (--nopush)."
fi
