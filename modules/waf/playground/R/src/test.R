#!/usr/bin/env Rscript

require(R.utils)
require(tools)

cmdArgs <- commandArgs(asValues = TRUE, excludeReserved = FALSE, excludeEnvVars = TRUE, os = "current")

if(!is.null(cmdArgs[["numTrees"]])) nTrees <- (as.integer(cmdArgs[["numTrees"]])) else nTrees <- 500

ffff <- cmdArgs[["ffff"]]
print(paste("nTrees =", nTrees, sep = " "))

Sweave(ffff)
texi2dvi("testSweave.tex", pdf = TRUE)

Stangle(ffff)
source("testSweave.R")

print(paste("Current Date/Time: ", date(), sep=""))

(sessionInfoObj <- sessionInfo())

