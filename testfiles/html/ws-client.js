        var OnLeft = false
        var OnRight = false;
        var OnForward = false;
        var OnBack = false;
        var msgCnt = 0;
        var lastCmd = '0';
        var lastData = '';
        var sndCmdCnt = 0;
        var sndDataCnt = 0;
        var tmpState = false;

$(document).ready(function(){

//      var WEBSOCKET_ROUTE = "/ws";
        var WEBSOCKET_ROUTE = "";

	const queryString = window.location.search;
	const urlParams = new URLSearchParams(queryString);
	var host = urlParams.get('host')
	if ( host == null || host == '' )
	    host = window.location.hostname;

//	console.log("host");
//	console.log(host);

        if(window.location.protocol == "http:"){
            //localhost
            var ws = new WebSocket("ws://" + host + ":9090" + WEBSOCKET_ROUTE);
        }
        else if(window.location.protocol == "https:"){
            //Dataplicity
            var ws = new WebSocket("wss://" + host + ":9090" + WEBSOCKET_ROUTE);
        }

        var myVar = setInterval(function() {
           if ( sndCmdCnt > 0 ) {
              sndCmdCnt--
              ws.send('{"op":"publish","topic":"/xbox","msg":{"data":"C' + msgCnt.toString(16) + lastCmd + '"}}');
              msgCnt = (msgCnt+1) % 16;
           }

           if ( sndDataCnt > 0 ) {
              sndDataCnt--
              ws.send('{"op":"publish","topic":"/xbox","msg":{"data":"D' + msgCnt.toString(16) + lastData + '"}}');
              msgCnt = (msgCnt+1) % 16;
           }
        }, 100 );

        ws.onopen = function(evt) {
            $("#ws-status").html("Connected");
            };

        ws.onmessage = function(evt) {
            };

        ws.onclose = function(evt) {
            $("#ws-status").html("Disconnected");
            };

        $("#speed1").click(function(){
            if ( lastCmd != '1' ) {
              lastCmd = '1';
              sndCmdCnt = 3;
              }
            });

        $("#speed2").click(function(){
            if ( lastCmd != '2' ) {
              lastCmd = '2';
              sndCmdCnt = 3;
              }
            });

        $("#speed3").click(function(){
            if ( lastCmd != '3' ) {
              lastCmd = '3';
              sndCmdCnt = 3;
              }
            });

        $("#btnmotor").click(function(){
            if ( lastCmd != 'A' ) {
              lastCmd = 'A';
              sndCmdCnt = 3;
              }
            });

        $("#lighton").click(function(){
            if ( lastCmd != 'D' ) {
              lastCmd = 'D';
              sndCmdCnt = 3;
              }
            });

        $("#lightoff").click(function(){
            if ( lastCmd != 'E' ) {
              lastCmd = 'E';
              sndCmdCnt = 3;
              }
            });

        $("#rptvlton").click(function(){
            if ( lastCmd != 'B' ) {
              lastCmd = 'B';
              sndCmdCnt = 3;
              }
            });

        $("#rptvltoff").click(function(){
            if ( lastCmd != 'C' ) {
              lastCmd = 'C';
              sndCmdCnt = 3;
              }
            });

        $("#fogon").click(function(){
            if ( lastCmd != 'F' ) {
              lastCmd = 'F';
              sndCmdCnt = 3;
              }
            });

        $("#fogoff").click(function(){
            if ( lastCmd != 'G' ) {
              lastCmd = 'G';
              sndCmdCnt = 3;
              }
            });

        $("#d_left").click(function(){
            tmpState = OnLeft;
            ResetBtns();
            if ( !tmpState ) {
              OnLeft = true;
              this.className = "btnBlue";
              lastData = "8040"
              sndDataCnt = 3;
            }
            });

        $("#d_right").click(function(){
            tmpState = OnRight;
            ResetBtns();
            if ( !tmpState ) {
              OnRight = true;
              this.className = "btnBlue";
              lastData = "80C0"
              sndDataCnt = 3;
            }
            });

        $("#d_forward").click(function(){
            tmpState = OnForward;
            ResetBtns();
            if ( !tmpState ) {
              OnForward = true;
              this.className = "btnBlue";
              lastData = "F080"
              sndDataCnt = 3;
            }
            });

        $("#d_back").click(function(){
            tmpState = OnBack;
            ResetBtns();
            if ( !tmpState ) {
              OnBack = true;
              this.className = "btnBlue";
              lastData = "1080"
              sndDataCnt = 3;
            }
            });

        $("#d_stop").click(function(){
              ResetBtns();
            });

        document.addEventListener('keydown', (event) => {
            console.log(`KEYDOWN: key=${event.key},code=${event.code}`);
            if ( event.key == '1' && lastCmd != '1') {
              lastCmd = '1';
              sndCmdCnt = 3;
            }
            if ( event.key == '2' && lastCmd != '2') {
              lastCmd = '2';
              sndCmdCnt = 3;
            }
            if ( event.key == 'a' && lastCmd != 'A') {
              lastCmd = 'A';
              sndCmdCnt = 3;
            }
            if ( event.key == 'b' && lastCmd != 'B') {
              lastCmd = 'B';
              sndCmdCnt = 3;
            }
            if ( event.key == 'c' && lastCmd != 'C') {
              lastCmd = 'C';
              sndCmdCnt = 3;
            }
            if ( event.key == 'd' && lastCmd != 'D') {
              lastCmd = 'D';
              sndCmdCnt = 3;
            }
            if ( event.key == 'e' && lastCmd != 'E') {
              lastCmd = 'E';
              sndCmdCnt = 3;
            }
            if ( event.key == 'f' && lastCmd != 'F') {
              lastCmd = 'F';
              sndCmdCnt = 3;
            }
            if ( event.key == 'g' && lastCmd != 'G') {
              lastCmd = 'G';
              sndCmdCnt = 3;
            }

            if ( event.key == 'ArrowUp' && lastData != 'F080') {
              ResetBtns();
              lastData = 'F080';
              sndDataCnt = 3;
            }
            if ( event.key == 'ArrowLeft' && lastData != '8040') {
              ResetBtns();
              lastData = '8040';
              sndDataCnt = 3;
            }
            if ( event.key == 'ArrowRight' && lastData != '80C0') {
              ResetBtns();
              lastData = '80C0';
              sndDataCnt = 3;
            }
            if ( event.key == 'ArrowDown' && lastData != '1080') {
              ResetBtns();
              lastData = '1080';
              sndDataCnt = 3;
            }
        });

        document.addEventListener('keyup', (event) => {

            if ( event.key == 'ArrowDown' || event.key == 'ArrowUp' || event.key == 'ArrowLeft' || event.key == 'ArrowRight' ) {
               ResetBtns();
            }
            console.log(`KEYUP: key=${event.key},code=${event.code}`);
        });


      });

function ResetBtns() {
    lastData = '8080';
    sndDataCnt = 3;
              OnLeft = false;
              OnRight = false;
              OnForward = false;
              OnBack = false;
    document.getElementById("d_left").className="";
    document.getElementById("d_right").className="";
    document.getElementById("d_forward").className="";
    document.getElementById("d_back").className="";
}
