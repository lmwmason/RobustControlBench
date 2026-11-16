FIGURE_SOURCE := ../results/figures
FIGURES := nominal-temperature controller-input disturbance-rejection capacity-uncertainty heat-transfer-uncertainty actuator-saturation measurement-noise stability-domain-summary original-vs-retuned higher-fidelity-summary
PDF_FIGURES := $(addprefix figures/,$(addsuffix .pdf,$(FIGURES)))
.PHONY: all figures clean
all: chemical-control-robustness.pdf
figures: $(PDF_FIGURES)
figures/%.pdf: $(FIGURE_SOURCE)/%.svg
	@mkdir -p figures
	rsvg-convert --format=pdf --output=$@ $<
chemical-control-robustness.pdf: figures main.tex references.bib $(wildcard sections/*.tex) $(wildcard tables/*.tex)
	SOURCE_DATE_EPOCH=1794820000 FORCE_SOURCE_DATE=1 latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
	cp build/main.pdf $@
clean:
	latexmk -C main.tex
	rm -rf build chemical-control-robustness.pdf figures/*.pdf
