<!DOCTYPE html>
<html>
    <head>
        <meta charset="utf-8">
    </head>
    <body>

	
		Your name is: 	  <b><?php echo $_GET["name"]; ?>	</b>	<br>
		Your password is: <b><?php echo $_GET["password"]; ?>	</b><br>
		
		<p>
		Check Box 1:		 <b><?php echo $_GET["fruit1"]; ?></b><br>
		Check Box 2:		 <b><?php echo $_GET["fruit2"]; ?></b>
		</p>
		
		<p>Which animal: 	 <b><?php echo $_GET["animal"]; ?>	</b></p>
		
		<p>Selected file: 	 <b><?php echo $_GET["file"]; ?>	</b></p>
		
		<p>Hidden value: 	 <b><?php echo $_GET["hidden"]; ?>	</b></p>
		
		<p>Your region: 	 <b><?php echo $_GET["city"]; ?>	</b></p>
		
		<p>Your sentence: 	 <b><?php echo $_GET["txtarea"]; ?>	</b></p>
	
		
    </body>
</html>