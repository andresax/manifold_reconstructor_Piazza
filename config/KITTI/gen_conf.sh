echo cp "KITTI_mono_noBA.json" $1"/KITTI"$1"_mono_noBA.json"
cp "KITTI_mono_noBA.json" $1"/KITTI"$1"_mono_noBA.json"

echo sed -i -- "s/€1/"$1"/g" $1"/KITTI"$1"_mono_noBA.json"
sed -i -- "s/€1/"$1"/g" $1"/KITTI"$1"_mono_noBA.json"


