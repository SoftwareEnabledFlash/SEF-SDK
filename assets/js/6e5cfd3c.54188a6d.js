"use strict";(self.webpackChunkweb_doc=self.webpackChunkweb_doc||[]).push([[317],{3905:(e,t,n)=>{n.d(t,{Zo:()=>d,kt:()=>f});var r=n(7294);function o(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function a(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function i(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?a(Object(n),!0).forEach((function(t){o(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):a(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function l(e,t){if(null==e)return{};var n,r,o=function(e,t){if(null==e)return{};var n,r,o={},a=Object.keys(e);for(r=0;r<a.length;r++)n=a[r],t.indexOf(n)>=0||(o[n]=e[n]);return o}(e,t);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);for(r=0;r<a.length;r++)n=a[r],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(o[n]=e[n])}return o}var c=r.createContext({}),s=function(e){var t=r.useContext(c),n=t;return e&&(n="function"==typeof e?e(t):i(i({},t),e)),n},d=function(e){var t=s(e.components);return r.createElement(c.Provider,{value:t},e.children)},p="mdxType",u={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},m=r.forwardRef((function(e,t){var n=e.components,o=e.mdxType,a=e.originalType,c=e.parentName,d=l(e,["components","mdxType","originalType","parentName"]),p=s(n),m=o,f=p["".concat(c,".").concat(m)]||p[m]||u[m]||a;return n?r.createElement(f,i(i({ref:t},d),{},{components:n})):r.createElement(f,i({ref:t},d))}));function f(e,t){var n=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var a=n.length,i=new Array(a);i[0]=m;var l={};for(var c in t)hasOwnProperty.call(t,c)&&(l[c]=t[c]);l.originalType=e,l[p]="string"==typeof e?e:o,i[1]=l;for(var s=2;s<a;s++)i[s]=n[s];return r.createElement.apply(null,i)}return r.createElement.apply(null,n)}m.displayName="MDXCreateElement"},6388:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>c,contentTitle:()=>i,default:()=>u,frontMatter:()=>a,metadata:()=>l,toc:()=>s});var r=n(7462),o=(n(7294),n(3905));const a={sidebar_position:2},i="Extending SEF CLI",l={unversionedId:"CLI/extending-cli",id:"CLI/extending-cli",title:"Extending SEF CLI",description:"chap-cliExtending}",source:"@site/docs/CLI/extending-cli.md",sourceDirName:"CLI",slug:"/CLI/extending-cli",permalink:"/SEF-SDK/CLI/extending-cli",draft:!1,tags:[],version:"current",sidebarPosition:2,frontMatter:{sidebar_position:2},sidebar:"docs",previous:{title:"SEF CLI Targets",permalink:"/SEF-SDK/CLI/cli-targets"},next:{title:"Flexible I/O Tester (FIO)",permalink:"/SEF-SDK/FIO/overview"}},c={},s=[{value:"DevTools",id:"subsec-cliDevTools",level:2}],d={toc:s},p="wrapper";function u(e){let{components:t,...n}=e;return(0,o.kt)(p,(0,r.Z)({},d,n,{components:t,mdxType:"MDXLayout"}),(0,o.kt)("h1",{id:"chap-cliExtending"},"Extending SEF CLI"),(0,o.kt)("p",null,"SEF-CLI was created with extensibility and improvement in mind; its functionality may be easily\nextended by adding new targets and by adding new actions to existing targets. Similarly, the\nfunctionality of existing actions may be easily updated or augmented. A good use case for the\nextensibility of SEF-CLI is configuring custom code without the need for creating a separate\napp. For example, SEF-CLI has been extended by adding an ",(0,o.kt)("inlineCode",{parentName:"p"},"ftl")," target to interact with the\nreference FTL module. In the same way, SEF-CLI may be extended to interact with user-defined\ntargets."),(0,o.kt)("p",null,"To get started, begin by examining the existing targets. The targets are located in the ",(0,o.kt)("inlineCode",{parentName:"p"},"engines"),"\ndirectory. Moreover, an example ",(0,o.kt)("inlineCode",{parentName:"p"},"null")," target ",(0,o.kt)("inlineCode",{parentName:"p"},"engines/null.c")," has been added, which can be used\nas a model or template for building another desired custom target."),(0,o.kt)("h2",{id:"subsec-cliDevTools"},"DevTools"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"devtool")," directory includes development-only code to generate the man page and the autocomplete\nscript. After making any changes to SEF-CLI, these tools may be used to generate a new\nman page and auto-complete script. In order to use the DevTools, the ",(0,o.kt)("inlineCode",{parentName:"p"},"DEVTOOL")," macro should be\ndefined. CMake option ",(0,o.kt)("inlineCode",{parentName:"p"},"sef_cli_enable_devtool")," will define the ",(0,o.kt)("inlineCode",{parentName:"p"},"DEVTOOL")," macro and enable\nuse of the DevTools."),(0,o.kt)("p",null,"The following commands enable the DevTools while building sef-cli and shows how to generate\nthe man page and the auto-complete script."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre"},"$ cd <path to SEF_SDK>/cli\n$ cmake -Dsef_enable_devtool=ON ..\n$ make\n$ ./sef-cli --man-page -V > ../sef-cli.1\n\n\n$ ./sef-cli --auto-complete > ../sef-cli_completion.sh\n")))}u.isMDXComponent=!0}}]);