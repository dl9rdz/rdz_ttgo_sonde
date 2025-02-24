#!/bin/bash

MYPATH=$PWD
# old/for Arduino builds
# FULLIMG=${MYPATH}/out.img
# UPDIMG=${MYPATH}/build/RX_FSK.ino.bin

# new/for pio build
FULLIMG=${MYPATH}/.pio/build/ttgo-lora32/firmware-image.bin
UPDIMG=${MYPATH}/.pio/build/ttgo-lora32/firmware.bin

setup_git() {
  git config --global user.email "dl9rdz@darc.de"
  git config --global user.name "dl9rdz (via automated build)"
  GITHUB_API_KEY=`cat ~/.github.api.key`
}
generate_website_index() {
  echo "<html><head>" > download.html
  echo "<meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">" >> download.html
  echo "<title>rdz_ttgo_sonde</title>" >> download.html
  echo '<link rel="stylesheet" href="/assets/css/style.css?v=a43710928bb200926b87aed147b540673ccb0378">' >> download.html
  echo "</head><body>" >> download.html
  echo '<div class="wrapper"><header><h1><a href="https://dl9rdz.github.io/rdz_ttgo_sonde/">rdz_ttgo_sonde</a></h1><p></p>' >> download.html
  echo '<p class="view"><a href="https://github.com/dl9rdz/rdz_ttgo_sonde">View the Project on GitHub <small>dl9rdz/rdz_ttgo_sonde</small></a></p>' >> download.html
  echo '</header><section><h1 id="rdz_ttgo_sonde">rdz_ttgo_sonde</h1>' >> download.html

  echo "<h2>Main repository (future...)</h2><ul>" >> download.html
  for i in `ls main|sort -r|grep -v update-info`; do
    TS=`git log main/$i | grep "Date:" | head -1 | awk '{$1="";$2="";$7="";print substr($0,3,length($0)-3)}'`
    if [ -z "$TS" ]; then TS=`date`; fi
    echo -e "<li><a href=\"main/$i\">$i</a> ($TS)</li>\n" >> download.html
  done
  echo "</ul>" >> download.html
  echo "<h2>Development repository (dev2)</h2><ul>" >> download.html
  for i in `ls dev2|sort -r|grep "\.bin"`; do
    TS=`git log dev2/$i | grep "Date:" | head -1 | awk '{$1="";$2="";$7="";print substr($0,3,length($0)-3)}'`
    if [ -z "$TS" ]; then TS=`date`; fi
    VERS=`basename $i -full.bin`
    CL=$(awk '{printf "%s; ", $0}' dev2/${VERS}-changelog.txt 2>/dev/null)
    echo "VERS $VERS: CL $CL"
    echo "<li><a href=\"dev2/$i\">$i</a> ($TS)" >> download.html
    if [ -n "${CL}" ]; then echo "<br>${CL%??}" >> download.html; fi
    echo -e "</li>\n" >> download.html
  done
  echo "</ul>" >> download.html

  echo "<h2>Master repository (old IDF environment)</h2><ul>" >> download.html
  for i in `ls master|sort -r|grep -v update-info`; do
    TS=`git log master/$i | grep "Date:" | head -1 | awk '{$1="";$2="";$7="";print substr($0,3,length($0)-3)}'`
    if [ -z "$TS" ]; then TS=`date`; fi
    echo -e "<li><a href=\"master/$i\">$i</a> ($TS)</li>\n" >> download.html;
  done
  echo "</ul><h2>Development repository (old IDF environment)</h2><ul>" >> download.html
  for i in `ls devel|sort -r|grep "\.bin"`; do
    TS=`git log devel/$i | grep "Date:" | head -1 | awk '{$1="";$2="";$7="";print substr($0,3,length($0)-3)}'`
    if [ -z "$TS" ]; then TS=`date`; fi
    VERS=`basename $i -full.bin`
    CL=`cat devel/${VERS}-changelog.txt 2>/dev/null`
    echo "VERS $VERS: CL $CL"
    echo "<li><a href=\"devel/$i\">$i</a> ($TS)" >> download.html
    if [ -n "${CL}" ]; then echo "<br>${CL}" >> download.html; fi
    echo -e "</li>\n" >> download.html
  done
  echo "</ul>
  <br>
  <p>Last latter/number of version number indicate SPI LittleFS file system version. If the first (upper-case)
   letter has changed, then this version is incompabible with prevision versions and you have to flash
   the full image. If the second part (number) has changed, then this version has some changes
   (e.g. internal web page layout, LCD/TFT display layout) in the file system which you will not get with
   a code-only (OTA or flashing update.bin) update, but it should not break anything.</p>
   </section></body></html>" >> download.html
  git add download.html
  git commit --amend --message "Build @ `date`"
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

setup_git
commit_website_files
generate_website_index
upload_files
