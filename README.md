sphinx-quickstart

sphinx-build -b html sourcedir builddir
sphinx-build -b html src build

make html

make latexpdf
