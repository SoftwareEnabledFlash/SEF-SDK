_sef-cli ()
{
local opts cur
cur="${COMP_WORDS[COMP_CWORD]}"
opts=""
if [[ ${COMP_CWORD} -eq 1 ]] ; then
opts="${opts} list"
opts="${opts} draw"
opts="${opts} info"
opts="${opts} create"
opts="${opts} delete"
opts="${opts} set-pslc"
opts="${opts} set-suspend-config"
opts="${opts} interactive"
opts="${opts} execute"
opts="${opts} list"
opts="${opts} info"
opts="${opts} list"
opts="${opts} info"
opts="${opts} create"
opts="${opts} delete"
opts="${opts} format"
opts="${opts} backup"
opts="${opts} restore"
opts="${opts} resize"
opts="${opts} label"
opts="${opts} info"
opts="${opts} configure"
opts="${opts} check"
fi
if [[ ${COMP_CWORD} -eq 2 ]] ; then
local action
action="${COMP_WORDS[1]}"
if [[ ${action} == "list" ]] ; then
opts="${opts} virtual-device"
fi
if [[ ${action} == "draw" ]] ; then
opts="${opts} virtual-device"
fi
if [[ ${action} == "info" ]] ; then
opts="${opts} virtual-device"
fi
if [[ ${action} == "create" ]] ; then
opts="${opts} virtual-device"
fi
if [[ ${action} == "delete" ]] ; then
opts="${opts} virtual-device"
fi
if [[ ${action} == "set-pslc" ]] ; then
opts="${opts} virtual-device"
fi
if [[ ${action} == "set-suspend-config" ]] ; then
opts="${opts} virtual-device"
fi
if [[ ${action} == "interactive" ]] ; then
opts="${opts} shell"
fi
if [[ ${action} == "execute" ]] ; then
opts="${opts} shell"
fi
if [[ ${action} == "list" ]] ; then
opts="${opts} sef-unit"
fi
if [[ ${action} == "info" ]] ; then
opts="${opts} sef-unit"
fi
if [[ ${action} == "list" ]] ; then
opts="${opts} qos-domain"
fi
if [[ ${action} == "info" ]] ; then
opts="${opts} qos-domain"
fi
if [[ ${action} == "create" ]] ; then
opts="${opts} qos-domain"
fi
if [[ ${action} == "delete" ]] ; then
opts="${opts} qos-domain"
fi
if [[ ${action} == "format" ]] ; then
opts="${opts} qos-domain"
fi
if [[ ${action} == "backup" ]] ; then
opts="${opts} qos-domain"
fi
if [[ ${action} == "restore" ]] ; then
opts="${opts} qos-domain"
fi
if [[ ${action} == "resize" ]] ; then
opts="${opts} qos-domain"
fi
if [[ ${action} == "label" ]] ; then
opts="${opts} qos-domain"
fi
if [[ ${action} == "info" ]] ; then
opts="${opts} ftl"
fi
if [[ ${action} == "configure" ]] ; then
opts="${opts} ftl"
fi
if [[ ${action} == "check" ]] ; then
opts="${opts} ftl"
fi
fi
if [[ ${COMP_CWORD} -gt 2 ]] ; then
local target
target="${COMP_WORDS[2]}"
case "$target" in
virtual-device)
opts="${opts} --sef-index"
opts="${opts} -s"
opts="${opts} --virtual-device-id"
opts="${opts} -v"
opts="${opts} --num-pslc-super-block"
opts="${opts} --suspend-max-time-per"
opts="${opts} --suspend-min-time-until"
opts="${opts} --suspend-max-interval"
opts="${opts} --channel-num"
opts="${opts} --bank-num"
opts="${opts} --repeat-num"
opts="${opts} --die-map"
opts="${opts} --super-block"
opts="${opts} --read-weight"
opts="${opts} --tiling-strategy"
opts="${opts} --verbose"
opts="${opts} -V"
opts="${opts} --force"
opts="${opts} -f"
;;
shell)
opts="${opts} --python-script"
opts="${opts} --script-pipe"
;;
sef-unit)
opts="${opts} --sef-index"
opts="${opts} -s"
opts="${opts} --verbose"
opts="${opts} -V"
;;
qos-domain)
opts="${opts} --sef-index"
opts="${opts} -s"
opts="${opts} --qos-domain-id"
opts="${opts} -q"
opts="${opts} --label"
opts="${opts} -l"
opts="${opts} --virtual-device-id"
opts="${opts} -v"
opts="${opts} --flash-capacity"
opts="${opts} --flash-capacity-percent"
opts="${opts} --pslc-flash-capacity"
opts="${opts} --flash-quota"
opts="${opts} --adu-size"
opts="${opts} --api"
opts="${opts} --recovery"
opts="${opts} --defect-strategy"
opts="${opts} --encryption-key"
opts="${opts} --num-placement-id"
opts="${opts} --max-open-super-blocks"
opts="${opts} --read-queue"
opts="${opts} --program-weight"
opts="${opts} --erase-weight"
opts="${opts} --verbose"
opts="${opts} -V"
opts="${opts} --path"
opts="${opts} --prefix"
opts="${opts} --allow-list"
opts="${opts} --block-list"
opts="${opts} --relabel"
opts="${opts} --force"
opts="${opts} -f"
;;
ftl)
opts="${opts} --sef-index"
opts="${opts} -s"
opts="${opts} --qos-domain-id"
opts="${opts} -q"
opts="${opts} --label"
opts="${opts} -l"
opts="${opts} --num-domains"
opts="${opts} --overprovisioning"
opts="${opts} --should-repair"
opts="${opts} --force"
opts="${opts} -f"
;;
esac
fi
COMPREPLY=( $(compgen -W "${opts}" -- ${cur}) )
return 0
}
complete -F _sef-cli sef-cli
