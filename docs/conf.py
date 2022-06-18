# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
import codecs

# Import a script to read files and get a list of all imported modules
sys.path.insert(0, './scripts')
from utils import get_all_imports
sys.path.remove('./scripts')

sys.path.insert(
    0, os.path.abspath("../src/simple_ur_move")
)  # Add the simple_ur_move src path to to add things to autodoc




# -- Project information -----------------------------------------------------

project = "simple_ur_move"
copyright = "2022, Clark B. Teeple, Harvard Microrobotics Lab"
author = "Clark B. Teeple, Harvard Microrobotics Lab"


# -- General configuration ---------------------------------------------------

# Set name of Master Doc
master_doc = "index"

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [  #'recommonmark',
    "sphinx_markdown_tables",
    "sphinx_copybutton",
    "sphinx.ext.autodoc",
    "sphinx.ext.coverage",
    "sphinx.ext.napoleon",
    "sphinx.ext.graphviz",
    "sphinx.ext.mathjax",
    "sphinxcontrib.bibtex",
    "sphinx_panels",
    "m2r2",
]


bibtex_bibfiles = ["refs.bib"]
bibtex_default_style = "bibtexlabels"
bibtex_reference_style = "author_year"


import pybtex.plugin
from pybtex.style.formatting.unsrt import Style as UnsrtStyle
from pybtex.style.labels import BaseLabelStyle
from pybtex.plugin import register_plugin


class MyLabelStyle(BaseLabelStyle):
    def format_labels(self, sorted_entries):
        for entry in sorted_entries:
            yield entry.key


class MyStyle(UnsrtStyle):
    default_label_style = MyLabelStyle


register_plugin("pybtex.style.formatting", "bibtexlabels", MyStyle)


source_suffix = [".rst", ".md"]

# Create fake imports for autodoc to work
autodoc_folders_to_use = ['../src/simple_ur_move']
autodoc_mock_imports = []
for folder in autodoc_folders_to_use:
    for root, dirs, files in os.walk(folder):
        for name in files:
            if name.endswith('.py'):
                curr_imports = list(get_all_imports(os.path.join(root, name)))
                autodoc_mock_imports.extend(curr_imports)

# Read in all required packages and add them to a list
# import requirements

# with open("../requirements.txt", "r") as fd:
#     for req in requirements.parse(fd):
#         autodoc_mock_imports.append(req.name)

autodoc_mock_imports = sorted(list(set(autodoc_mock_imports)))
print('\n'+"IMPORTING FAKE PACKAGES FOR AUTODOC: ")
for item in autodoc_mock_imports:
    print(item)
print("")

#add_module_names = False


# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

html_favicon = 'img/favicon.ico'
html_logo = "img/logo_wide.png"
html_theme_options = {
    "logo_only": False,
    "collapse_navigation": False,
    "sticky_navigation": True,
    "style_nav_header_background": "#fcfcfc",
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

html_css_files = [
    'css/custom.css',
]
