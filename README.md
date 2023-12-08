# Software-Enabled Flash Software Development Kit (SDK)

This repository contains the host-based SDK for Software-Enabled Flash (SEF) technology, including:

* Linux(tm) Kernel patches and SEF device driver
* Reference FTL implementation with Flexible Data Placement (FDP), Zoned Namespace (ZNS), NVMe, and basic block support
* Command Line Interface (CLI)
* QEMU paravirtualized SEF drivers
* Patches to nvme-cli enabling SEF support
* Patches to FIO enabling SEF support

Full documentation on using and understanding these components is included [here](SEF-SDK-01-00.pdf).

# License

The code and patches in this repository are licensed under the [BSD 3-Clause License](LICENSE.md).

Any submodules referenced by this repository, including `fio`, `linux`, `nvme-cli`, and `qemu` are copyright their respective owners and licensed under their respective terms.  Please see the submodule trees directly for their copyright and license terms.

# Getting Involved

For more information, please visit the [Software-Enabled Flash Project website](https://softwareenabledflash.org).

* This project follows the [Linux Foundation Antitrust Policy](https://www.linuxfoundation.org/legal/antitrust-policy).