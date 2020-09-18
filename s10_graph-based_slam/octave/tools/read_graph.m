% read a g2o data file describing a 2D SLAM instance
function graph = read_graph(filename)

fid = fopen(filename, 'r');

graph = struct (
  'x', [],
  'edges', [],
  'idLookup', struct
);

disp('Parsing File');
while true
  ln = fgetl(fid);
  if (ln == -1)
    break;
  end
  tokens = strsplit(ln, ' ', true);
  double_tokens = str2double(tokens);

  tk = 2;
  if (strcmp(tokens(1), 'VERTEX_SE2') != 0)
    id = int32(double_tokens(tk++));
    values = double_tokens(tk:tk+2)'; tk += 3;
    graph.idLookup = setfield(graph.idLookup, num2str(id), struct('offset', length(graph.x), 'dimension', length(values)));
    graph.x = [graph.x; values];
  elseif (strcmp(tokens(1), 'VERTEX_XY') != 0)
    id = int32(double_tokens(tk++));
    values = double_tokens(tk:tk+1)'; tk += 2;
    graph.idLookup = setfield(graph.idLookup, num2str(id), struct('offset', length(graph.x), 'dimension', length(values)));
    graph.x = [graph.x; values];
  elseif (strcmp(tokens(1), 'EDGE_SE2') != 0)
    fromId = int32(double_tokens(tk++));
    toId = int32(double_tokens(tk++));
    measurement = double_tokens(tk:tk+2)'; tk += 3;
    uppertri = double_tokens(tk:tk+5)'; tk += 6;
    information = [uppertri(1), uppertri(2), uppertri(3);
                   uppertri(2), uppertri(4), uppertri(5);
                   uppertri(3), uppertri(5), uppertri(6)];
    graph.edges = [graph.edges; struct(
      'type', 'P',
      'from', fromId,
      'to', toId,
      'measurement', measurement,
      'information', information)];
  elseif (strcmp(tokens(1), 'EDGE_SE2_XY') != 0)
    fromId = int32(double_tokens(tk++));
    toId = int32(double_tokens(tk++));
    measurement = double_tokens(tk:tk+1)'; tk += 2;
    uppertri = double_tokens(tk:tk+2)'; tk += 3;
    information = [uppertri(1), uppertri(2); uppertri(2), uppertri(3)];
    graph.edges = [graph.edges; struct(
      'type', 'L',
      'from', fromId,
      'to', toId,
      'measurement', measurement,
      'information', information)];
  end

end

% setup the index into the state vector
disp('Preparing helper structs');
for eid = 1:length(graph.edges)
  graph.edges(eid).fromIdx = getfield(graph.idLookup, num2str(graph.edges(eid).from)).offset + 1;
  graph.edges(eid).toIdx = getfield(graph.idLookup, num2str(graph.edges(eid).to)).offset + 1;
end

end
