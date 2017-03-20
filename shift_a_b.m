function shiftedLabImage=shift_a_b(a_shift, b_shift, labImage)
        
%         -- scale the chroma distance shifted according to amount of
%         -- luminance. The 1.1 overshoot is because we cannot be sure
%         -- to have gotten the data in the first place.
%         a_delta = a_shift * (l/100) * 1.1;
%         b_delta = b_shift * (l/100) * 1.1;
        shiftedLabImage=labImage;
        shiftedLabImage(:,:,2)=labImage(:,:,2)+a_shift*(labImage(:,:,1)/100)*1.1;
        shiftedLabImage(:,:,3)=labImage(:,:,3)+b_shift*(labImage(:,:,1)/100)*1.1;
  
end