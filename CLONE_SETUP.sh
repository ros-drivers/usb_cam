cat .gitmodules| sed 's|git@github.com:agcooke/|git://github.com/agcooke/|g' > tmp
mv tmp .gitmodules
