function Formation_mat = GenAngleFormation()
    all_index = 1:8;
    for i = 3:8
        Formation_mat{i} = nchoosek(all_index,i);
    end
end

