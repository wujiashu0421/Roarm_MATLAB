%Script that runs the different SPART tests and produces a code-coverage
%report.

%--- Clean and Clear ---%
clc
clear
close all

%--- Configure code coverage report ---%
import matlab.unittest.TestRunner
import matlab.unittest.plugins.CodeCoveragePlugin
runner = TestRunner.withTextOutput;
runner.addPlugin(CodeCoveragePlugin.forFolder('../src/','IncludingSubfolders',true))

%--- Define test suite ---%
suite = testsuite({'CompareTest','FixedJointsTest','CoMTest'});

%--- Run tests ---%
result = runner.run(suite);