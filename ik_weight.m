function weight = ik_weight(markerInputMap)

% weights
weight(markerInputMap('Sternum')) = 1;
weight(markerInputMap('LAcromium')) = 0.5;
weight(markerInputMap('TopHead')) = 0.1;
weight(markerInputMap('RASIS')) = 10;
weight(markerInputMap('LASIS')) = 10;
weight(markerInputMap('VSacral')) = 10; 
weight(markerInputMap('RThighUpper')) = 1;
weight(markerInputMap('RThighFront')) = 1;
weight(markerInputMap('RThighRear')) = 1;
weight(markerInputMap('RShankUpper')) = 1;
weight(markerInputMap('RShankFront')) = 1;
weight(markerInputMap('RShankRear')) = 1;
weight(markerInputMap('RHeel')) = 10;
weight(markerInputMap('RMidfootSup')) = 1;
weight(markerInputMap('RMidfootLat')) = 1;
weight(markerInputMap('RToeLat')) = 1;
weight(markerInputMap('RToeMed')) = 1;
weight(markerInputMap('LThighUpper')) = 1;
weight(markerInputMap('LThighFront')) = 1;
weight(markerInputMap('LThighRear')) = 1;
weight(markerInputMap('LShankUpper')) = 1;
weight(markerInputMap('LShankFront')) = 1;
weight(markerInputMap('LShankRear')) = 1;
weight(markerInputMap('LHeel')) = 10;
weight(markerInputMap('LMidfootSup')) = 1;
weight(markerInputMap('LMidfootLat')) = 1;
weight(markerInputMap('LToeLat')) = 1;
weight(markerInputMap('LToeMed')) = 1;

end