xacro --inorder rrbot.xacro gazebo:=true > rrbot.xml
gz sdf --print rrbot.xml > rrbot.sdf
