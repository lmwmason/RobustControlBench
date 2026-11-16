FIGURE_SOURCE := ../results/figures
FIGURES := nominal-temperature controller-input disturbance-rejection capacity-uncertainty heat-transfer-uncertainty actuator-saturation measurement-noise stability-domain-summary original-vs-retuned higher-fidelity-summary
PDF_FIGURES := $(addprefix figures/,$(addsuffix .pdf,$(FIGURES)))
.PHONY: all figures korean clean
all: chemical-control-robustness.pdf chemical-control-robustness-ko.pdf
korean: chemical-control-robustness-ko.pdf
figures: $(PDF_FIGURES)
figures/%.pdf: $(FIGURE_SOURCE)/%.svg
	@mkdir -p figures
	SOURCE_DATE_EPOCH=1794820000 rsvg-convert --format=pdf --output=$@ $<
chemical-control-robustness.pdf: figures main.tex references.bib $(wildcard sections/*.tex) $(wildcard tables/*.tex)
	SOURCE_DATE_EPOCH=1794820000 FORCE_SOURCE_DATE=1 latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
	cp build/main.pdf $@
chemical-control-robustness-ko.pdf: figures main-ko.tex references.bib $(wildcard sections-ko/*.tex) $(wildcard tables/*.tex) fonts/NotoSansKR.ttf
	@mkdir -p build-ko
	SOURCE_DATE_EPOCH=1794820000 FORCE_SOURCE_DATE=1 latexmk -lualatex -interaction=nonstopmode -halt-on-error -outdir=build-ko main-ko.tex
	cp build-ko/main-ko.pdf $@
clean:
	latexmk -C main.tex
	rm -rf build build-ko chemical-control-robustness.pdf chemical-control-robustness-ko.pdf figures/*.pdf
