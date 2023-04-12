function Ps = simplify_polygon(P)
  % Handle degenerate cases!
  if size(P,1)<3
      Ps = P;
      return;
  end
  % Normalise the difference between successive points
  pdiff = diff(P([end-1 1:end],:)); % Append the 2nd-to-last
  npdiff = bsxfun(@rdivide, pdiff, sqrt(sum(pdiff.^2,2)));
  % Any successive normalised differences that don't change can be discarded
  keepIdxs = find(any(diff(npdiff),2));
  % Append the first/last points (the question states that P(1,:)==P(end,:))
  Ps = P(keepIdxs([1:end 1]),:);
end