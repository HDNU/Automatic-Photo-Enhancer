function remapImage=remap(v, min1, max1)
    remapImage= (v-min1) * 1.0/(max1-min1);
end