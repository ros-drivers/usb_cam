cat .gitmodules| sed 's|git@github.com:agcooke/|git://github.com/agcooke/|g' > tmp
mv tmp .gitmodules
git submodule init
git submodule update
cd sofiehdfformat_rosdriver/
rosmake
