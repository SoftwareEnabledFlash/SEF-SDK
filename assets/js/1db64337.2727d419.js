"use strict";(self.webpackChunkweb_doc=self.webpackChunkweb_doc||[]).push([[372],{3905:(e,t,n)=>{n.d(t,{Zo:()=>p,kt:()=>f});var r=n(7294);function i(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function o(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function a(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?o(Object(n),!0).forEach((function(t){i(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function s(e,t){if(null==e)return{};var n,r,i=function(e,t){if(null==e)return{};var n,r,i={},o=Object.keys(e);for(r=0;r<o.length;r++)n=o[r],t.indexOf(n)>=0||(i[n]=e[n]);return i}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(r=0;r<o.length;r++)n=o[r],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(i[n]=e[n])}return i}var l=r.createContext({}),c=function(e){var t=r.useContext(l),n=t;return e&&(n="function"==typeof e?e(t):a(a({},t),e)),n},p=function(e){var t=c(e.components);return r.createElement(l.Provider,{value:t},e.children)},u="mdxType",m={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},d=r.forwardRef((function(e,t){var n=e.components,i=e.mdxType,o=e.originalType,l=e.parentName,p=s(e,["components","mdxType","originalType","parentName"]),u=c(n),d=i,f=u["".concat(l,".").concat(d)]||u[d]||m[d]||o;return n?r.createElement(f,a(a({ref:t},p),{},{components:n})):r.createElement(f,a({ref:t},p))}));function f(e,t){var n=arguments,i=t&&t.mdxType;if("string"==typeof e||i){var o=n.length,a=new Array(o);a[0]=d;var s={};for(var l in t)hasOwnProperty.call(t,l)&&(s[l]=t[l]);s.originalType=e,s[u]="string"==typeof e?e:i,a[1]=s;for(var c=2;c<o;c++)a[c]=n[c];return r.createElement.apply(null,a)}return r.createElement.apply(null,n)}d.displayName="MDXCreateElement"},6777:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>l,contentTitle:()=>a,default:()=>m,frontMatter:()=>o,metadata:()=>s,toc:()=>c});var r=n(7462),i=(n(7294),n(3905));const o={sidebar_position:2},a="Overview",s={unversionedId:"overview",id:"overview",title:"Overview",description:"chap-Overview}",source:"@site/docs/overview.md",sourceDirName:".",slug:"/overview",permalink:"/SEF-SDK/overview",draft:!1,tags:[],version:"current",sidebarPosition:2,frontMatter:{sidebar_position:2},sidebar:"docs",previous:{title:"Intended Audience",permalink:"/SEF-SDK/intended-audience"},next:{title:"Definitions and Acronyms",permalink:"/SEF-SDK/definitions"}},l={},c=[{value:"Figure 1: SEF Components Diagram",id:"fig-SefComponentDiagram",level:4}],p={toc:c},u="wrapper";function m(e){let{components:t,...o}=e;return(0,i.kt)(u,(0,r.Z)({},p,o,{components:t,mdxType:"MDXLayout"}),(0,i.kt)("h1",{id:"chap-Overview"},"Overview"),(0,i.kt)("p",null,"The Software-Enabled Flash\u2122\ufe0f (SEF) Software Development Kit (SDK) is distributed in source\ncode format and consists of the following major components:"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},"SEF API: The SEF API is the Application Programming Interface, it represents a contract\nbetween an application and the implementation. The implementation is free to change as long\nas the API remains constant."),(0,i.kt)("li",{parentName:"ul"},'SEF Library: The SEF Library is a user mode library with a set of functions that may be\nused to implement the SEF API, translating the functions defined in the API into the actual\ncommands issued to the device. To do so, the library also implements many "helper functions"\nthat are not defined by the API, but are of general utility in either implementing the API\nor working around device specific restrictions/limitations. There is no contract between the\nlibrary and the application for anything that is not a part of the API, and these helper\nfunctions are free to change without affecting applications. An example of a helper function\nis a function that manipulates or copies a scatter gather list, assisting in the cases where a\nsingle command from the perspective of the API needs to be broken up into multiple device\ncommands due to device specific limitations. The SEF Library encompasses more than just\nthe API; the SEF Library provides quite a bit of information on how to use and interact with\nthe SEF Command Set.'),(0,i.kt)("li",{parentName:"ul"},"SEF Reference FTL: A user mode FTL built on the SEF Library. It exposes the asynchronous\nSEF Block API."),(0,i.kt)("li",{parentName:"ul"},"SEF CLI: A command line tool for listing and configuring most aspects of an SEF Unit. It\nsupports multiple target types, such as SEF Units, Virtual Devices, QoS Domains, and SEF\nFTL block devices."),(0,i.kt)("li",{parentName:"ul"},"SEF FIO: Provides an SEF FTL block device ioengine to the Linux I/O testing tool fio."),(0,i.kt)("li",{parentName:"ul"},"SEF QEMU Block Driver: Provides three SEF-backed block device types to QEMU: a virtual\nblock device, a block NVMe\u2122\ufe0f device, and a ZNS NVMe device."),(0,i.kt)("li",{parentName:"ul"},"SEF Kernel Driver: Builds the sef.ko module"),(0,i.kt)("li",{parentName:"ul"},"SEF Linux: Includes the hooks required to use an SEF Unit"),(0,i.kt)("li",{parentName:"ul"},"SEF NVME-CLI: Provides support for SEF subcommands.")),(0,i.kt)("p",null,"Figure ",(0,i.kt)("a",{parentName:"p",href:"#fig-SefComponentDiagram"},"1")," shows how the components fit together."),(0,i.kt)("h4",{id:"fig-SefComponentDiagram"},"Figure 1: SEF Components Diagram"),(0,i.kt)("p",null,(0,i.kt)("img",{alt:"SEF Components Diagram",src:n(5387).Z,width:"961",height:"415"})),(0,i.kt)("p",null,"The goal of the SEF SDK is to provide a software set that enables the immediate evaluation of SEF\ntechnology as well as to illustrate the use of the SEF API and SEF Library to build real world\nstorage applications. ",(0,i.kt)("strong",{parentName:"p"},"Although the SEF SDK has been extensively tested, it is not intended\nfor commercial use in a production environment.")," The SEF API is documented in the ",(0,i.kt)("em",{parentName:"p"},"Software\nEnabled Flash\u2122\ufe0f (SEF) API Specification"),"."),(0,i.kt)("p",null,"This document is NOT intended to cover the design and implementation of the SEF Library,\nmodifications to the NVMe device driver, the SEF Unit driver or the SEF utility programs. The\nsource code for all of these components is available as part of the SDK."))}m.isMDXComponent=!0},5387:(e,t,n)=>{n.d(t,{Z:()=>r});const r=n.p+"assets/images/component-diagram-69ab906a1d85b9cbe82b1ca4c507af88.png"}}]);