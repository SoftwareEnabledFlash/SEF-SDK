"use strict";(self.webpackChunkweb_doc=self.webpackChunkweb_doc||[]).push([[181],{3905:(e,t,a)=>{a.d(t,{Zo:()=>c,kt:()=>f});var n=a(7294);function r(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function o(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(e);t&&(n=n.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,n)}return a}function s(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?o(Object(a),!0).forEach((function(t){r(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):o(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function i(e,t){if(null==e)return{};var a,n,r=function(e,t){if(null==e)return{};var a,n,r={},o=Object.keys(e);for(n=0;n<o.length;n++)a=o[n],t.indexOf(a)>=0||(r[a]=e[a]);return r}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(n=0;n<o.length;n++)a=o[n],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(r[a]=e[a])}return r}var d=n.createContext({}),l=function(e){var t=n.useContext(d),a=t;return e&&(a="function"==typeof e?e(t):s(s({},t),e)),a},c=function(e){var t=l(e.components);return n.createElement(d.Provider,{value:t},e.children)},h="mdxType",u={inlineCode:"code",wrapper:function(e){var t=e.children;return n.createElement(n.Fragment,{},t)}},p=n.forwardRef((function(e,t){var a=e.components,r=e.mdxType,o=e.originalType,d=e.parentName,c=i(e,["components","mdxType","originalType","parentName"]),h=l(a),p=r,f=h["".concat(d,".").concat(p)]||h[p]||u[p]||o;return a?n.createElement(f,s(s({ref:t},c),{},{components:a})):n.createElement(f,s({ref:t},c))}));function f(e,t){var a=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var o=a.length,s=new Array(o);s[0]=p;var i={};for(var d in t)hasOwnProperty.call(t,d)&&(i[d]=t[d]);i.originalType=e,i[h]="string"==typeof e?e:r,s[1]=i;for(var l=2;l<o;l++)s[l]=a[l];return n.createElement.apply(null,s)}return n.createElement.apply(null,a)}p.displayName="MDXCreateElement"},2284:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>d,contentTitle:()=>s,default:()=>u,frontMatter:()=>o,metadata:()=>i,toc:()=>l});var n=a(7462),r=(a(7294),a(3905));const o={sidebar_position:5},s="Persistence",i={unversionedId:"FTL/persistence",id:"FTL/persistence",title:"Persistence",description:"chap-Persistence}",source:"@site/docs/FTL/persistence.md",sourceDirName:"FTL",slug:"/FTL/persistence",permalink:"/SEF-SDK/FTL/persistence",draft:!1,tags:[],version:"current",sidebarPosition:5,frontMatter:{sidebar_position:5},sidebar:"docs",previous:{title:"Garbage Collection",permalink:"/SEF-SDK/FTL/garbage-collection"},next:{title:"Instrumentation",permalink:"/SEF-SDK/FTL/instrumentation"}},d={},l=[{value:"Data Management",id:"data-management",level:2},{value:"Data to Be Stored",id:"data-to-be-stored",level:3},{value:"Stored Data",id:"stored-data",level:3},{value:"Root Pointer Management",id:"root-pointer-management",level:3},{value:"Data Tree Object",id:"data-tree-object",level:2},{value:"Figure 5: Data Tree Object Structure",id:"fig-DTOStructure",level:4}],c={toc:l},h="wrapper";function u(e){let{components:t,...o}=e;return(0,r.kt)(h,(0,n.Z)({},c,o,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"chap-Persistence"},"Persistence"),(0,r.kt)("p",null,"The SEF Reference FTL uses the persistence layer to store the flash translation layer\u2019s operating\nmetadata, such as the lookup table, and Super Block information to the flash memory. The\npersistence layer manages that in two parts: Data Management and Data Tree storage."),(0,r.kt)("h2",{id:"data-management"},"Data Management"),(0,r.kt)("p",null,"The data management code, which is located in ",(0,r.kt)("inlineCode",{parentName:"p"},"persistence.c"),", is used to manage the data to\nbe stored, stored data, and the root pointer."),(0,r.kt)("h3",{id:"data-to-be-stored"},"Data to Be Stored"),(0,r.kt)("p",null,"After the persistence layer is initialized, individual components of the Reference FTL can queue\ndata to be flushed. The data is stored and identified by a unique key and index. While queuing\nthe data, the caller may provide a callback function that will be called when the data is flushed to\nthe flash memory or when the persistence cleanup is called. This callback function may be used to\nclean up allocated memory or perform other follow-up actions. Moreover, given the return value of\nisFlushed, appropriate action may be taken considering whether the data was flushed."),(0,r.kt)("p",null,"Queuing the data does not ensure the data is stored to the flash memory. In order to store the data\non the flash memory, the queued data should be flushed by calling ",(0,r.kt)("inlineCode",{parentName:"p"},"PDLFlushToFlash()"),". The\ncaller is responsible for freeing the data previously stored on the flash memory. In other words,\nthe previously written data should be cleared manually by calling ",(0,r.kt)("inlineCode",{parentName:"p"},"PDLFreeFlash()")," to avoid\nleaks."),(0,r.kt)("h3",{id:"stored-data"},"Stored Data"),(0,r.kt)("p",null,"In order to keep track of the flushed data, a metadata table is used. The metadata table is an array\nof stored objects and their unique keys, unique indexes, sizes, and CRC-32. The array of metadata\nis flushed at the time of storage and is used to identify and read the stored data."),(0,r.kt)("p",null,"As part of persistence initialization, the persistence layer uses the root pointers to locate the\nmetadata table and find out about the stored data. A cyclic redundancy check (CRC-32) is used\nin order to verify the stored metadata. Moreover, given that multiple code bases may use the\npersistence layer independent of the Reference FTL, each code base should use a Globally Unique\nIdentifier (GUID). The GUID is checked at the time of persistence initialization."),(0,r.kt)("p",null,"To read the stored data, the caller uses a unique key and index. The stored data is also verified\nusing the CRC-32."),(0,r.kt)("p",null,"In order to separate the user data from the metadata stored by persistence, the persistence layer\nstores the data on isolated Super Blocks. The persistence layer manually allocates Super Blocks\nthat are only used by persistence to store the queued data."),(0,r.kt)("h3",{id:"root-pointer-management"},"Root Pointer Management"),(0,r.kt)("p",null,"The root pointers are used to keep track of the persisted data. They are used as pointers into the\ndevice where the metadata table is stored. If the root pointer is null, it is assumed that the device\nis empty and no persistence data has been flushed; however, a null root pointer could also indicate\nthat the device is not empty and the persistence layer was not able to store the metadata or update\nthe root pointer. In order to avoid ambiguity, the caller should mark the root pointer as dirty after\ndata has been written to the device."),(0,r.kt)("p",null,"Given that the Reference FTL may experience unexpected shutdowns, the root pointer is an\nimportant tool for verifying the freshness of the read data. To detect an unclean shutdown, the root\npointer is checked at initialization time. A dirty root pointer denotes that the software experienced\nan unexpected shutdown."),(0,r.kt)("p",null,"The root pointer is set to clean either automatically, after data is flushed to the flash memory, or\nmanually, by being marked as clean."),(0,r.kt)("p",null,"The root pointer is marked as dirty by creating a Flash Address that has its QoS Domain ID set to\nzero. The root pointer can be marked as clean and reconstructed by setting the QoS Domain ID of\nthe Flash Address back to the QoS Domain ID of the loaded QoS Domain. The Reference FTL uses\nthis mechanism in order to detect an unexpected shutdown."),(0,r.kt)("p",null,"In summary, the root pointer is checked when the Reference FTL is initialized, and a dirty root\npointer denotes the occurrence of an unexpected shutdown. If the root pointer is clean, the Reference\nFTL marks the root pointer as dirty when the first write is performed. When the Reference FTL\ngoes through the expected shutdown process, the data is flushed and a clean root pointer is set,\ndenoting a clean shutdown."),(0,r.kt)("h2",{id:"data-tree-object"},"Data Tree Object"),(0,r.kt)("p",null,"The Data Tree Object (DTO) is a self-contained data structure used to store contiguous data. The\nneed for a DTO becomes apparent given SEF\u2019s support for various defect-management methods\nthat do not guarantee contiguous data placement. The kFragmented method, for example, may not\nstore data contiguously."),(0,r.kt)("p",null,"The DTO root is always 1 ADU in size and stores the flash addresses of the next tree nodes. For\nexample, if an ADU is 4096 bytes and the DTO node structure is 16 bytes, then 510 flash addresses\n(8 bytes each) can be used to store either the locations of the next layer\u2019s nodes or the location\nof data. Given this structure, the root node can store the flash addresses for up to 2 megabytes of data."),(0,r.kt)("p",null,"Figure ",(0,r.kt)("a",{parentName:"p",href:"#fig-DTOStructure"},"5")," illustrates the structure of an example DTO. In this example, just\nby having a two-layer tree, persistence is able to keep track of an object with a maximum size of\n1065 megabytes."),(0,r.kt)("h4",{id:"fig-DTOStructure"},"Figure 5: Data Tree Object Structure"),(0,r.kt)("p",null,(0,r.kt)("img",{alt:"Data Tree Object Structure",src:a(3746).Z,width:"958",height:"155"})),(0,r.kt)("p",null,"The DTO offers functions to read, write, free, and get the size of an entire tree in order to simplify\nuse of the given tree. Moreover, the host can get the Super Blocks used by a given tree. Given the\nstructure of the DTO, it must be read as a whole and not partially."))}u.isMDXComponent=!0},3746:(e,t,a)=>{a.d(t,{Z:()=>n});const n=a.p+"assets/images/persistence-data-tree-object-3cdf6f0780977fd8e42de3ab93ca818a.png"}}]);