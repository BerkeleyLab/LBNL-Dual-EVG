# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build
STATICDIR     = _static

ELEMENTS = \
	ChassisBlockDiagram.png \
	Display.png  \
	EVG2clk.png  \
	EVGfront.png \
	Digilent_PMODGPS.png \
	EVG1clk.png \
	EVGback.png \
	MarbleBlockDiagram.png \
	Coincidence.svg

STATIC_ELEMENTS = $(addprefix $(STATICDIR)/, $(ELEMENTS))
BUILD_ELEMENTS = $(addprefix $(BUILDDIR)/html/, $(ELEMENTS))


# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help html clean Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
html: Makefile $(BUILDDIR)/html $(BUILD_ELEMENTS)
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

$(BUILDDIR)/html:
	mkdir -p $@

_build/html/%.png: _static/%.png
	install -m 755 $^ $@

_build/html/%.svg: _static/%.svg
	install -m 755 $^ $@

clean::
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

