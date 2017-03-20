function [avg_a,avg_b]=get_avg_a_b (labImage,width, height)


%first find average color in CIE Lab space
sum_a=sum(sum(labImage(:,:,2)));
sum_b=sum(sum(labImage(:,:,3)));

avg_a=sum_a/(width*height);
avg_b=sum_b/(width*height);

end