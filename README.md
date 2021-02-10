# License

*OpenUxAS* is developed by the Air Force Research Laboratory, Aerospace System Directorate, Power and Control Division.
The LMCP specification and all source code for *OpenUxAS* is publicaly released under the Air Force Open Source Agreement Version 1.0. See LICENSE.md for complete details.
The Air Force Open Source Agreement closely follows the NASA Open Source Agreement Verion 1.3.
**NOTE the terms of the license include registering use of the software by emailing <a href="mailto:afrl.rq.opensource@us.af.mil?subject=OpenUxAS Registration&body=Please register me for use of OpenUxAS. Name: ____________">afrl.rq.opensource@us.af.mil</a>.**

OpenUxAS
========

***Note:** This repository is work in progress on integration of OpenUxAS-bootstrap into OpenUxAS.
The README is thus incomplete and only includes notes for testing.*

# Getting Started #

Clone this repository normally, placing it wherever you like.
Commands will be given relative to the root of this repository, like this:

    OpenUxAS$ echo $PWD

In all of the examples, we will be specifying maximum verbosity with `-vv`.
This is to illustrate the debug-level output provided by the various scripts.
In particular, all of the scripts **print the command(s) they execute**.
This should help users get unstuck if the automation breaks down for some reason.

Once the clone is complete, try to run make:

    OpenUxAS$ make -j all

This will fail — which is expected: make is trying to access includes from libraries that have not yet been installed.
To fix this, we need to build OpenUxAS with anod:

    OpenUxAS$ ./anod -vv build uxas

Since this is the first time anod is run, it will automatically install its infrastructure.
Once the infrastructure is installed, the build will start.

When the build is complete, make can be re-run:

    OpenUxAS$ make -j all

Before examples can be run (successfully), OpenAMASE should be built:

    OpenUxAS$ ./anod -vv build amase

Now, examples can be run:

    OpenUxAS$ ./run-example -vv 02_Example_WaterwaySearch

Note that run-example is very clear about which OpenAMASE and which OpenUxAS binary is being used.
Note also that the commands run by run-example are clearly printed.

To run the Ada examples:

    OpenUxAS$ ./anod -vv build uxas-ada

Then:

    OpenUxAS$ ./run-example -vv 02a_Ada_WaterwaySearch

Again, note that run-example is clear about which uxas-ada binary is being used.

# Things to Try #

You can reset your sandbox with:

    OpenUxAS$ ./anod -vv reset

***Note:** this does what it says on the box: resets (deletes) your sandbox.
You will then have to rebuild.*

You can clean your local C++ build with `make clean`.
Then try running the example again:

    OpenUxAS$ ./run-example -vv 02_Example_WaterwaySearch

Assuming you didn't also just reset your anod sandbox, you'll see that run-example is now using your anod-built OpenUxAS binary.

You can run tests with:

    OpenUxAS/tests/cpp$ ./run-tests -vv

Note that the script is very clear about the commands it runs and which OpenUxAS it runs.
To force use of the sandbox OpenUxAS, even if there is a locally-built OpenUxAS (from `make -j all`), try:

    OpenUxAS/tests/cpp$ ./run-tests -vv --qualifier=scenario=release

You can also redo the OpenUxAS build passing the coverage qualifier:

    OpenUxAS$ ./anod -vv build uxas --qualifier=scenario=gcov

Then run the test suite with the same qualifier:

    OpenUxAS/tests/cpp$ ./run-tests -vv --qualifier=scenario=gcov

You can control which version of OpenUxAS is used by way of the qualifier.

Finally, you can run the proofs using the proof script, as before:

    OpenUxAS/tests/proof$ ./run-proofs -vv

(These will fail unless you have first built uxas-ada using anod, as expected.)
Note that the script is very clear about the commands it runs.

Finally, as before, there is a devel-setup command that is appropriate for both OpenAMASE and LmcpGen:

    OpenUxAS$ ./anod -vv devel-setup amase lcmp
