ADPSL Releases
==============

The latest untagged master branch can be obtained at
https://github.com/areaDetector/ADPSL.

Tagged source code and pre-built binary releases prior to R2-0 are included
in the areaDetector releases available via links at
http://cars.uchicago.edu/software/epics/areaDetector.html.

Tagged source code releases from R2-0 onward can be obtained at 
https://github.com/areaDetector/ADPSL/releases.

Tagged prebuilt binaries from R2-0 onward can be obtained at
http://cars.uchicago.edu/software/pub/ADPSL.

The versions of EPICS base, asyn, and other synApps modules used for each release can be obtained from 
the EXAMPLE_RELEASE_PATHS.local, EXAMPLE_RELEASE_LIBS.local, and EXAMPLE_RELEASE_PRODS.local
files respectively, in the configure/ directory of the appropriate release of the 
[top-level areaDetector](https://github.com/areaDetector/areaDetector) repository.



Release Notes
=============

R2-1 (July 4, 2017)
----
* Many improvements to the driver.  The initial work was done by Adam Bark from Diamond.
  - Worked with Perceval Guillou at Photonic Science to improve the protocol with the server so it is more
    consistent and easier to parse.  Version 4c3 or later of their server is required to work with
    this release of the EPICS driver. 
    Photonic Science says that all of their cameras will work with the new server.  
    New DLLs will be needed in some cases.
  - Added a CameraName record to select the camera to control.
  - Implemented the asynEnum interface so that mbbo/mbbi records (CameraName, TriggerMode, FileFormat)
    get their choices from a list provided by the server.  This guarantees that the choices will be valid
    for a specific installation and the selected camera.
  - Added support for color cameras.
  - Improved the report() function.
* Finished the PSLDoc.html documentation, converting from marCCD document.

R2-0 (March 20, 2014)
----
* Moved the repository to [Github](https://github.com/areaDetector/ADPSL).
* Re-organized the directory structure to separate the driver library from the example IOC application.
* Added timestamps to NDArrays, this was previously overlooked for this driver

R1-9-1 and earlier
------------------
Release notes are part of the
[areaDetector Release Notes](http://cars.uchicago.edu/software/epics/areaDetectorReleaseNotes.html).
