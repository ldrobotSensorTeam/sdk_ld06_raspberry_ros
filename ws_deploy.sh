#!/bin/bash
#Author: David Hu
#Date: 2020-12
floder_name=$1
null_name=" "
if [ ${floder_name} == ${null_name} ]
then
    echo "please input \"./ws_deploy.sh floder_name\""
else
    floder_name="${floder_name}_`date +%Y%m%d-%H-%M`"
    mkdir ${floder_name}
    cp ./*.md ./${floder_name}
    cp ./rviz ./${floder_name} -a
    cp ./src  ./${floder_name} -a
    zip -r ${floder_name}.zip ./${floder_name}
    mv ./${floder_name} ../
    echo "mv ./${floder_name} ../"
    mv ./${floder_name}.zip ../
    echo "mv ./${floder_name}.zip ../"
fi
