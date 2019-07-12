<?php

	// Load JSON state
    $string = file_get_contents("robotStateNew.json");
    $json_a= json_decode($string,true);

	//first part is the name of the value in the json text file that will be read by server.php when called from Arduino
	//second part is the URL value that is parsed.
    $json_a['mode'] = $_GET["URLmode"];
    $json_a['xval'] = $_GET["URLxval"];
    $json_a['yval'] = $_GET["URLyval"];
	$json_a['udlr'] = $_GET["URLudlr"];
	$json_a['cmdVal'] = $_GET["URLcmdVal"];

    $fp = fopen('robotStateNew.json', 'w');
    fwrite($fp, json_encode($json_a));
    fclose($fp);

?>


