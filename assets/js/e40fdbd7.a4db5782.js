"use strict";(self.webpackChunkweb_doc=self.webpackChunkweb_doc||[]).push([[502],{3905:(e,n,t)=>{t.d(n,{Zo:()=>d,kt:()=>h});var l=t(7294);function a(e,n,t){return n in e?Object.defineProperty(e,n,{value:t,enumerable:!0,configurable:!0,writable:!0}):e[n]=t,e}function i(e,n){var t=Object.keys(e);if(Object.getOwnPropertySymbols){var l=Object.getOwnPropertySymbols(e);n&&(l=l.filter((function(n){return Object.getOwnPropertyDescriptor(e,n).enumerable}))),t.push.apply(t,l)}return t}function r(e){for(var n=1;n<arguments.length;n++){var t=null!=arguments[n]?arguments[n]:{};n%2?i(Object(t),!0).forEach((function(n){a(e,n,t[n])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(t)):i(Object(t)).forEach((function(n){Object.defineProperty(e,n,Object.getOwnPropertyDescriptor(t,n))}))}return e}function o(e,n){if(null==e)return{};var t,l,a=function(e,n){if(null==e)return{};var t,l,a={},i=Object.keys(e);for(l=0;l<i.length;l++)t=i[l],n.indexOf(t)>=0||(a[t]=e[t]);return a}(e,n);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(l=0;l<i.length;l++)t=i[l],n.indexOf(t)>=0||Object.prototype.propertyIsEnumerable.call(e,t)&&(a[t]=e[t])}return a}var s=l.createContext({}),u=function(e){var n=l.useContext(s),t=n;return e&&(t="function"==typeof e?e(n):r(r({},n),e)),t},d=function(e){var n=u(e.components);return l.createElement(s.Provider,{value:n},e.children)},c="mdxType",p={inlineCode:"code",wrapper:function(e){var n=e.children;return l.createElement(l.Fragment,{},n)}},m=l.forwardRef((function(e,n){var t=e.components,a=e.mdxType,i=e.originalType,s=e.parentName,d=o(e,["components","mdxType","originalType","parentName"]),c=u(t),m=a,h=c["".concat(s,".").concat(m)]||c[m]||p[m]||i;return t?l.createElement(h,r(r({ref:n},d),{},{components:t})):l.createElement(h,r({ref:n},d))}));function h(e,n){var t=arguments,a=n&&n.mdxType;if("string"==typeof e||a){var i=t.length,r=new Array(i);r[0]=m;var o={};for(var s in n)hasOwnProperty.call(n,s)&&(o[s]=n[s]);o.originalType=e,o[c]="string"==typeof e?e:a,r[1]=o;for(var u=2;u<i;u++)r[u]=t[u];return l.createElement.apply(null,r)}return l.createElement.apply(null,t)}m.displayName="MDXCreateElement"},6486:(e,n,t)=>{t.r(n),t.d(n,{assets:()=>s,contentTitle:()=>r,default:()=>p,frontMatter:()=>i,metadata:()=>o,toc:()=>u});var l=t(7462),a=(t(7294),t(3905));const i={sidebar_position:5},r="Build Instructions",o={unversionedId:"build-instructions",id:"build-instructions",title:"Build Instructions",description:"chap-BuildInstructions}",source:"@site/docs/build-instructions.md",sourceDirName:".",slug:"/build-instructions",permalink:"/SEF-SDK/build-instructions",draft:!1,tags:[],version:"current",sidebarPosition:5,frontMatter:{sidebar_position:5},sidebar:"docs",previous:{title:"Security Model",permalink:"/SEF-SDK/security-model"},next:{title:"SEF Driver",permalink:"/SEF-SDK/Driver/overview"}},s={},u=[{value:"General Build Requirements",id:"general-build-requirements",level:2},{value:"Source Code",id:"source-code",level:2},{value:"Linux\u2122\ufe0f Kernel",id:"linux\ufe0f-kernel",level:2},{value:"SEF Driver",id:"sec-sefDriver",level:2},{value:"SEF Library, Flash Translation Layer (FTL) and Command Line Interface (CLI)",id:"sef-library-flash-translation-layer-ftl-and-command-line-interface-cli",level:2},{value:"FIO",id:"fio",level:2},{value:"SEF QEMU",id:"sef-qemu",level:2},{value:"NVME-CLI",id:"nvme-cli",level:2},{value:"Validating Installation",id:"validating-installation",level:2}],d={toc:u},c="wrapper";function p(e){let{components:n,...t}=e;return(0,a.kt)(c,(0,l.Z)({},d,t,{components:n,mdxType:"MDXLayout"}),(0,a.kt)("h1",{id:"chap-BuildInstructions"},"Build Instructions"),(0,a.kt)("h2",{id:"general-build-requirements"},"General Build Requirements"),(0,a.kt)("p",null,"This chapter will cover how to get the Software Development Kit (SDK) source, build and install\ndifferent components. Specific requirements and dependencies are listed for each component along\nwith the install command. The Software-Enabled Flash\u2122\ufe0f (SEF) SDK components and tools were\ndesigned to run on 32-bit or 64-bit systems and are endian agnostic."),(0,a.kt)("p",null,(0,a.kt)("em",{parentName:"p"},"NOTE: The build instructions below match how the SDK is built on Ubuntu\u2122\ufe0f 20.04")),(0,a.kt)("h2",{id:"source-code"},"Source Code"),(0,a.kt)("p",null,"The SEF SDK will soon be available to clone from GitHub. The following command can be used to\nclone with SEF SDK:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ git clone https://github.com/SoftwareEnabledFlash/SEF-SDK.git\n")),(0,a.kt)("p",null,(0,a.kt)("em",{parentName:"p"},"Note: Pre-release distributions will be distributed as compressed tar images which may be extracted\nas follows:")),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ tar -xzvf SEFSDK.tar.gz\n")),(0,a.kt)("h2",{id:"linux\ufe0f-kernel"},"Linux\u2122\ufe0f Kernel"),(0,a.kt)("p",null,"SEF SDK requires a modified Linux Kernel 5.19. This section covers how to download, modify,\nbuild and install the required kernel. To learn more about SEF Linux and how it works refer to\n",(0,a.kt)("a",{parentName:"p",href:"/SEF-SDK/Driver/overview#chap-driver"},"SEF Linux\u2122\ufe0f Driver")," Chapter."),(0,a.kt)("p",null,(0,a.kt)("em",{parentName:"p"},"Note: Cloning the Linux repository as part of the build will take several minutes.")),(0,a.kt)("p",null,"Before getting started, ensure the following prerequisites have been installed:"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},"libncurses-dev"),(0,a.kt)("li",{parentName:"ul"},"gawk"),(0,a.kt)("li",{parentName:"ul"},"flex"),(0,a.kt)("li",{parentName:"ul"},"bison"),(0,a.kt)("li",{parentName:"ul"},"openssl"),(0,a.kt)("li",{parentName:"ul"},"libssl-dev"),(0,a.kt)("li",{parentName:"ul"},"libudev-dev"),(0,a.kt)("li",{parentName:"ul"},"dwarves"),(0,a.kt)("li",{parentName:"ul"},"zstd"),(0,a.kt)("li",{parentName:"ul"},"libelf-dev"),(0,a.kt)("li",{parentName:"ul"},"libpci-dev"),(0,a.kt)("li",{parentName:"ul"},"libiberty-dev"),(0,a.kt)("li",{parentName:"ul"},"autoconf"),(0,a.kt)("li",{parentName:"ul"},"dkms"),(0,a.kt)("li",{parentName:"ul"},"bc")),(0,a.kt)("p",null,"The following command can be used to install these dependencies:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ sudo apt update\n$ sudo apt install -y libncurses-dev gawk flex bison openssl \\\n libssl-dev dwarves zstd libelf-dev libpci-dev libiberty-dev \\\n autoconf dkms bc\n")),(0,a.kt)("p",null,"Because this is a custom kernel, you may receive an error about a bad signature, which requires\nSecure Boot disabled in BIOS."),(0,a.kt)("p",null,"We provide a kioxia.config built from a server install of Ubuntu\u2122\ufe0f 20.04\u2019s config. However, you\ncan create your own .config file with the \u2018make menuconfig\u2018 command."),(0,a.kt)("p",null,"To build and install the SEF Linux, issue the following commands:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ cd <path to SEF_SDK>/linux\n$ ./apply_patch.sh\n$ cd official-linux\n$ cp ./arch/x86/configs/kioxia.config .config\n$ make -j`nproc`\n$ sudo make -j`nproc` INSTALL_MOD_STRIP=1 modules_install\n$ sudo make install\n$ sudo reboot now\n")),(0,a.kt)("h2",{id:"sec-sefDriver"},"SEF Driver"),(0,a.kt)("p",null,"It is important to note that the SEF Driver and the Linux kernel must be built on the same\nsystem to ensure compatibility. The sef.ko kernel module requires a system running SEF Linux\nin order to access the SEF Unit. To learn more about SEF Driver and how it works refer to\n",(0,a.kt)("a",{parentName:"p",href:"/SEF-SDK/Driver/overview#chap-driver"},"SEF Linux\u2122\ufe0f Driver")," Chapter."),(0,a.kt)("p",null,(0,a.kt)("em",{parentName:"p"},"Note: If the driver fails to load with a message regarding unknown symbols, it is because nvme-core.ko\nand nvme_ko were not previously loaded by the system. Also, please check if your SEF\nUnit was properly installed using lspci command.")),(0,a.kt)("p",null,"Before getting started, ensure the following prerequisites have been installed:"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},"build-essential"),(0,a.kt)("li",{parentName:"ul"},"libudev-dev"),(0,a.kt)("li",{parentName:"ul"},"cmake")),(0,a.kt)("p",null,"The following command can be used to install these dependencies:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ sudo apt install -y build-essential libudev-dev cmake\n")),(0,a.kt)("p",null,"To build and install the SEF Driver so that it will load across reboots, first build Linux, then issue\nthe following commands:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ cd <path to SEF_SDK>\n$ export SEF_KERNEL_SRC=/<path to SEF_SDK>/linux/official-linux/\n$ sudo SEF_KERNEL_SRC=${SEF_KERNEL_SRC} ./build.sh -o driver -i\n$ sudo modprobe sef\n")),(0,a.kt)("h2",{id:"sef-library-flash-translation-layer-ftl-and-command-line-interface-cli"},"SEF Library, Flash Translation Layer (FTL) and Command Line Interface (CLI)"),(0,a.kt)("p",null,"The SEF SDK uses the cmake and make tool set in order to prepare, build, and install the SDK\ncomponents. The Software Development Kit can be downloaded and installed in one easy step. The\nmaster SEF SDK project includes all the SDK modules."),(0,a.kt)("p",null,"To learn more about SEF Library, FTL, and CLI and\nhow they work, refer to ",(0,a.kt)("a",{parentName:"p",href:"/SEF-SDK/Library/overview#chap-SEFLibrary"},"SEF Library")," Chapter,\n",(0,a.kt)("a",{parentName:"p",href:"/SEF-SDK/FTL/overview#chap-SefFtl"},"SEF Reference Flash Translation Layer (FTL)")," Chapter,\n",(0,a.kt)("a",{parentName:"p",href:"/SEF-SDK/CLI/overview#chap-cli"},"Command Line Interface (CLI)")," Chapter respectively."),(0,a.kt)("p",null,"Before getting started, ensure the following prerequisites have been installed:"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},"cmake"),(0,a.kt)("li",{parentName:"ul"},"python3-dev (python3.6m or higher)")),(0,a.kt)("p",null,"The following command can be used to install these dependencies:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ sudo apt install -y cmake python3-dev\n")),(0,a.kt)("p",null,"To build the SEF Library, FTL, and CLI, issue the following commands:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ cd <path to SEF_SDK>\n$ sudo ./build.sh -i\n")),(0,a.kt)("p",null,"As part of the installation, the process will automatically install the CLI\u2019s man page and auto-complete script."),(0,a.kt)("h2",{id:"fio"},"FIO"),(0,a.kt)("p",null,"To learn more about SEF FIO Engine and how it works refer to\n",(0,a.kt)("a",{parentName:"p",href:"/SEF-SDK/FIO/overview#chap-fio"},"Flexible I/O Tester (FIO)")," Chapter."),(0,a.kt)("p",null,"In addition to the dependencies of FIO, ensure the following SEF prerequisites have been in-\nstalled:"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},"SEF Reference FTL library"),(0,a.kt)("li",{parentName:"ul"},"SEF library"),(0,a.kt)("li",{parentName:"ul"},"Header and libraries for pthread")),(0,a.kt)("p",null,"Before building FIO, it should be configured. The SEF Engine is enabled by default. To configure,\nbuild, and install FIO, issue the following commands:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ cd <path to SEF_SDK>/fio\n$ ./configure\n$ make\n$ sudo make install\n")),(0,a.kt)("h2",{id:"sef-qemu"},"SEF QEMU"),(0,a.kt)("p",null,"Although QEMU can be used to test software that has not be changed to support SEF natively, it\nis not required to use or test SEF. To learn more about SEF QEMU and how it works refer to\n",(0,a.kt)("a",{parentName:"p",href:"/SEF-SDK/QEMU/overview#chap-qemu"},"QEMU")," Chapter."),(0,a.kt)("p",null,"In addition to the dependencies of QEMU, ensure the following SEF prerequisites have been\ninstalled:"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},"SEF Reference FTL library"),(0,a.kt)("li",{parentName:"ul"},"SEF library"),(0,a.kt)("li",{parentName:"ul"},"Header and libraries for pthread"),(0,a.kt)("li",{parentName:"ul"},"git"),(0,a.kt)("li",{parentName:"ul"},"libglib2.0-dev"),(0,a.kt)("li",{parentName:"ul"},"libfdt-dev"),(0,a.kt)("li",{parentName:"ul"},"libpixman-1-dev"),(0,a.kt)("li",{parentName:"ul"},"zlib1g-dev"),(0,a.kt)("li",{parentName:"ul"},"ninja-build"),(0,a.kt)("li",{parentName:"ul"},"pkg-config")),(0,a.kt)("p",null,"The following command can be used to install these dependencies:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ sudo apt install -y git libpthread-stubs0-dev libglib2.0-dev \\\n libfdt-dev libpixman-1-dev zlib1g-dev ninja-build pkg-config\n")),(0,a.kt)("p",null,"SEF QEMU supports an SEF-backed virtual block device, block NVMe device, ZNS NVMe device\nor FDP."),(0,a.kt)("p",null,"To build and install the SEF QEMU, issue the following commands:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ cd <path to SEF_SDK>\n$ sudo -v; ./build.sh -o qemu -e -i\n")),(0,a.kt)("h2",{id:"nvme-cli"},"NVME-CLI"),(0,a.kt)("p",null,"Although NVME-CLI is a powerful tool that can be used to interact with and configure an\nSEF Unit, it is not required. To learn more about SEF NVME-CLI and how it works refer\nto ",(0,a.kt)("a",{parentName:"p",href:"/SEF-SDK/NVMe-CLI/overview#chap-NvmeCli"},"NVMe-CLI")," Chapter."),(0,a.kt)("p",null,"Ensure the following prerequisites have been installed:"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},"libjson-c-dev"),(0,a.kt)("li",{parentName:"ul"},"pthread"),(0,a.kt)("li",{parentName:"ul"},"meson"),(0,a.kt)("li",{parentName:"ul"},"ninja-build")),(0,a.kt)("p",null,"The following command can be used to install these dependencies:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ sudo apt install -y meson ninja-build\n")),(0,a.kt)("p",null,"To build and install the NVME-CLI, issue the following commands:"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},"$ cd <path to SEF_SDK>/nvme-cli\n$ ./apply_patch.sh\n$ cd official-nvme\n$ meson .build\n$ ninja -C .build\n$ sudo make install\n")),(0,a.kt)("h2",{id:"validating-installation"},"Validating Installation"),(0,a.kt)("p",null,"Once the kernel, driver, libraries, and applications are built and installed, the following sequence of\ncommands can verify communications with an installed SEF Unit."),(0,a.kt)("p",null,"The following commands check if the SEF Unit can be detected by listing all SEF Units, create a\nVirtual Device, create a QoS Domain, configure the FTL, and runs a short FIO job. The script\nassumes the SEF Unit is installed at index 0. The commands use the --force / -f flag to force the\nactions without asking for confirmation and the --verbose / -V flag to use the verbose mode. To\nlearn more about the commands being executed, read ",(0,a.kt)("a",{parentName:"p",href:"/SEF-SDK/CLI/overview#chap-cli"},"Command Line Interface (CLI)"),"\nChapter."),(0,a.kt)("p",null,(0,a.kt)("em",{parentName:"p"},"Note: The following instructions assume the driver has been loaded.")," ",(0,a.kt)("em",{parentName:"p"},"Prerequisite:\nShould have an empty SEF Unit. Delete Virtual Devices following directions in Section\n",(0,a.kt)("a",{parentName:"em",href:"/SEF-SDK/CLI/cli-targets#subsec-virtualDeviceDelete"},"Deleting Virtual Devices"))),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre"},'$ sudo sef-cli list sef\nSEF Unit Index           Vendor       FW Version      HW Version            Channels        Dies\nSEF Unit 0               KIOXIA       1LMSS528        1                     8               192\nSEF Unit Count 1\n\n\n$ sudo sef-cli create virtual-device --sef-index 0 \\\n --virtual-device-id 1 --super-block 1:32 --force --verbose\nDie Map:\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\n1    1    1    1    1    1    1    1\nThe Virtual Devices for SEF Unit 0 was successfully created\n\n\n$ sudo sef-cli create qos-domain --sef-index 0 --label               \\\n    "613006, 88877" --virtual-device-id 1 --force --defect-strategy  \\\n    "kPacked" --verbose --flash-capacity-percent 5\nThe QoS Domain 1 was successfully created\n\n\n$ sudo sef-cli configure ftl --sef-index 0 --label "613006, 88877"   \\\n    --force --verbose\nThe QoS Domain 1 was successfully configured by SDK\n\n$ sudo fio <path to SEF_SDK>/fio/official-fio/examples/sef.fio\n')))}p.isMDXComponent=!0}}]);