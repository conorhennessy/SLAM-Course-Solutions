% returns the block of the state vector which corresponds to the given id
function block = get_block_for_id(g, id)

blockInfo = getfield(g.idLookup, num2str(id));
block = g.x(1+blockInfo.offset : blockInfo.offset + blockInfo.dimension);

end
