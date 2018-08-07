git add .
var=$(zenity --entry --text="Insert your comment, it will apear in Github!" --title="Commit comment")
zenity --progress --pulsate
git commit -m $var
git push
zenity --progress --percentage=100 --auto-close
